// 先验占据栅格背景过滤节点
//
// 原理:
//   启动时把静态背景点云(与 ICP 目标同一份 PCD,同一坐标系)栅格化成 3D 占据体素,
//   并向外膨胀 dilation 格以吸收 ICP 残差与雷达测距噪声。
//   在线对 ICP 配准后的每一帧点云逐点查表:落入占据体素 -> 背景,否则 -> 前景。
//
// 数据流:
//   input_topic (/registration/icp, 已配准到目标坐标系的 PointXYZI)
//     -> 占据栅格查表
//     -> foreground_topic (前景, 下游分割用)
//     -> background_topic (背景, 仅 RViz 调试, 可关闭)

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/point_tests.h>  // pcl::isFinite
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class BackgroundFilterNode : public rclcpp::Node
{
public:
  BackgroundFilterNode()
  : Node("background_filter")
  {
    input_topic_      = this->declare_parameter<std::string>("input_topic", "/registration/icp");
    foreground_topic_ = this->declare_parameter<std::string>("foreground_topic", "/registration/foreground");
    background_topic_ = this->declare_parameter<std::string>("background_topic", "/registration/background");
    prior_file_       = this->declare_parameter<std::string>("prior_file", "");

    voxel_size_           = this->declare_parameter<double>("voxel_size", 0.1);
    dilation_             = this->declare_parameter<int>("dilation", 1);
    min_points_per_voxel_ = this->declare_parameter<int>("min_points_per_voxel", 1);
    publish_background_   = this->declare_parameter<bool>("publish_background", true);

    if (voxel_size_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "voxel_size 必须 > 0 (当前 %.3f)", voxel_size_);
      throw std::runtime_error("invalid voxel_size");
    }
    if (dilation_ < 0) dilation_ = 0;
    if (min_points_per_voxel_ < 1) min_points_per_voxel_ = 1;

    buildOccupancyGrid(prior_file_);

    const rclcpp::QoS qos = rclcpp::QoS(10);
    fg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(foreground_topic_, qos);
    if (publish_background_) {
      bg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(background_topic_, qos);
    }
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&BackgroundFilterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "背景过滤已启动: 订阅=%s 前景=%s 背景=%s | voxel=%.3fm dilation=%d",
      input_topic_.c_str(), foreground_topic_.c_str(),
      publish_background_ ? background_topic_.c_str() : "(关闭)",
      voxel_size_, dilation_);
  }

private:
  // 把世界坐标映射到栅格整数索引(向下取整)
  inline Eigen::Vector3i toIndex(float x, float y, float z) const
  {
    return Eigen::Vector3i(
      static_cast<int>(std::floor((x - origin_.x()) * inv_voxel_)),
      static_cast<int>(std::floor((y - origin_.y()) * inv_voxel_)),
      static_cast<int>(std::floor((z - origin_.z()) * inv_voxel_)));
  }

  inline bool inside(const Eigen::Vector3i & idx) const
  {
    return idx.x() >= 0 && idx.y() >= 0 && idx.z() >= 0 &&
           idx.x() < dims_.x() && idx.y() < dims_.y() && idx.z() < dims_.z();
  }

  inline std::size_t flatten(const Eigen::Vector3i & idx) const
  {
    return (static_cast<std::size_t>(idx.z()) * dims_.y() + idx.y()) * dims_.x() + idx.x();
  }

  void buildOccupancyGrid(const std::string & path)
  {
    pcl::PointCloud<pcl::PointXYZ> prior;
    if (path.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(path, prior) < 0 || prior.empty()) {
      RCLCPP_ERROR(this->get_logger(), "先验背景 PCD 加载失败或为空: '%s'", path.c_str());
      throw std::runtime_error("prior cloud is empty");
    }

    // 1) 包围盒(忽略 NaN/Inf),外扩 dilation 格,再各留 1 格余量
    Eigen::Vector3f min_pt(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt = -min_pt;
    std::size_t valid = 0;
    for (const auto & p : prior.points) {
      if (!pcl::isFinite(p)) continue;
      min_pt = min_pt.cwiseMin(p.getVector3fMap());
      max_pt = max_pt.cwiseMax(p.getVector3fMap());
      ++valid;
    }
    if (valid == 0) {
      RCLCPP_ERROR(this->get_logger(), "先验背景 PCD 中没有有效点");
      throw std::runtime_error("prior cloud has no finite points");
    }

    const float margin = static_cast<float>((dilation_ + 1) * voxel_size_);
    origin_    = (min_pt.array() - margin).matrix();
    inv_voxel_ = static_cast<float>(1.0 / voxel_size_);
    const Eigen::Vector3f extent = ((max_pt.array() + margin) - origin_.array()).matrix();
    dims_ = ((extent * inv_voxel_).array().ceil().cast<int>() + 1).matrix();

    const std::size_t total = static_cast<std::size_t>(dims_.x()) * dims_.y() * dims_.z();
    // 每体素 1 字节,防止误配置导致超大内存
    if (total > (1ull << 30)) {
      RCLCPP_ERROR(this->get_logger(),
        "栅格体素数 %zu 过大 (dims=%dx%dx%d),请增大 voxel_size", total, dims_.x(), dims_.y(), dims_.z());
      throw std::runtime_error("occupancy grid too large");
    }

    // 2) 点数统计 -> 阈值 -> 原始占据
    std::vector<uint16_t> counts(total, 0);
    for (const auto & p : prior.points) {
      if (!pcl::isFinite(p)) continue;
      const Eigen::Vector3i idx = toIndex(p.x, p.y, p.z);
      if (!inside(idx)) continue;
      uint16_t & c = counts[flatten(idx)];
      if (c < std::numeric_limits<uint16_t>::max()) ++c;
    }

    std::vector<uint8_t> raw(total, 0);
    std::size_t raw_occupied = 0;
    for (std::size_t i = 0; i < total; ++i) {
      if (counts[i] >= static_cast<uint16_t>(min_points_per_voxel_)) {
        raw[i] = 1;
        ++raw_occupied;
      }
    }
    counts.clear();
    counts.shrink_to_fit();

    // 3) 膨胀:对每个原始占据体素,把 (2d+1)^3 邻域标记为占据
    occupied_.assign(total, 0);
    if (dilation_ == 0) {
      occupied_.swap(raw);
    } else {
      for (int z = 0; z < dims_.z(); ++z) {
        for (int y = 0; y < dims_.y(); ++y) {
          for (int x = 0; x < dims_.x(); ++x) {
            if (!raw[flatten(Eigen::Vector3i(x, y, z))]) continue;
            for (int dz = -dilation_; dz <= dilation_; ++dz) {
              for (int dy = -dilation_; dy <= dilation_; ++dy) {
                for (int dx = -dilation_; dx <= dilation_; ++dx) {
                  const Eigen::Vector3i n(x + dx, y + dy, z + dz);
                  if (inside(n)) occupied_[flatten(n)] = 1;
                }
              }
            }
          }
        }
      }
    }

    std::size_t dilated_occupied = 0;
    for (const auto v : occupied_) dilated_occupied += v;

    RCLCPP_INFO(this->get_logger(),
      "先验占据栅格构建完成: %zu 个点 -> 栅格 %dx%dx%d (%zu 体素, %.1f MB) | "
      "占据 %zu -> 膨胀后 %zu | 原点=(%.2f, %.2f, %.2f)",
      valid, dims_.x(), dims_.y(), dims_.z(), total, total / 1048576.0,
      raw_occupied, dilated_occupied, origin_.x(), origin_.y(), origin_.z());
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) return;

    pcl::PointCloud<pcl::PointXYZI> foreground, background;
    foreground.reserve(cloud.size());
    if (publish_background_) background.reserve(cloud.size());

    for (const auto & p : cloud.points) {
      if (!pcl::isFinite(p)) continue;
      const Eigen::Vector3i idx = toIndex(p.x, p.y, p.z);
      // 栅格外的点一定不在先验背景里 -> 前景
      const bool is_bg = inside(idx) && occupied_[flatten(idx)];
      if (is_bg) {
        if (publish_background_) background.push_back(p);
      } else {
        foreground.push_back(p);
      }
    }

    sensor_msgs::msg::PointCloud2 fg_msg;
    pcl::toROSMsg(foreground, fg_msg);
    fg_msg.header = msg->header;
    fg_pub_->publish(fg_msg);

    if (publish_background_) {
      sensor_msgs::msg::PointCloud2 bg_msg;
      pcl::toROSMsg(background, bg_msg);
      bg_msg.header = msg->header;
      bg_pub_->publish(bg_msg);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "背景过滤 | 输入 %zu -> 前景 %zu (%.1f%%)",
      cloud.size(), foreground.size(), 100.0 * foreground.size() / cloud.size());
  }

  std::string input_topic_, foreground_topic_, background_topic_, prior_file_;
  double voxel_size_;
  int dilation_;
  int min_points_per_voxel_;
  bool publish_background_;

  // 稠密占据栅格(1 字节/体素),覆盖先验点云包围盒 + 膨胀余量
  Eigen::Vector3f origin_;
  Eigen::Vector3i dims_;
  float inv_voxel_ = 0.0f;
  std::vector<uint8_t> occupied_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bg_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BackgroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
