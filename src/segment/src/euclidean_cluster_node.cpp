// 欧式聚类节点 (分割流水线的一环)
//
// 原理 (pcl::EuclideanClusterExtraction):
//   仅按空间距离把点云分组:借助 KdTree 搜索,凡是两点欧式距离小于
//   cluster_tolerance 的点都会被归入同一个聚类(通过"距离可达"的方式
//   不断扩张)。再用聚类点数的上下限过滤掉过小的噪声和过大的背景/墙面。
//   参考: https://pointclouds.org/documentation/tutorials/cluster_extraction.html
//
// 数据流:
//   input_topic (sensor_msgs/PointCloud2, PointXYZI)
//     -> EuclideanClusterExtraction
//     -> 把每个聚类染上不同的 intensity 标签(便于在 RViz 中按强度着色)
//     -> 发布到 output_topic

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

class EuclideanClusterNode : public rclcpp::Node
{
public:
  EuclideanClusterNode()
  : Node("euclidean_cluster")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/preprocess/output");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/segment/clusters");

    // 空间距离阈值(米):两点欧式距离超过该值时一定不属于同一聚类。
    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 1.0);
    // 聚类点数的上下限:过小的视为噪声丢弃,过大的视为背景/墙面丢弃。
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 20);
    max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 25000);
    // 每帧打印每个簇的质心；生产环境可设为 false 降低日志量
    log_cluster_centers_ = this->declare_parameter<bool>("log_cluster_centers", true);

    if (cluster_tolerance_ <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(), "cluster_tolerance 必须大于 0，当前值: %.3f",
        cluster_tolerance_);
      throw std::invalid_argument("cluster_tolerance must be > 0");
    }
    if (min_cluster_size_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "min_cluster_size 不能为负，当前值: %d",
        min_cluster_size_);
      throw std::invalid_argument("min_cluster_size must be >= 0");
    }
    if (max_cluster_size_ < min_cluster_size_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "max_cluster_size(%d) 必须 >= min_cluster_size(%d)",
        max_cluster_size_, min_cluster_size_);
      throw std::invalid_argument("max_cluster_size must be >= min_cluster_size");
    }

    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&EuclideanClusterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "欧式聚类已启动: 订阅=%s 发布=%s | tolerance=%.3fm size=[%d, %d] log_centers=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      cluster_tolerance_, min_cluster_size_, max_cluster_size_,
      log_cluster_centers_ ? "true" : "false");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // 空帧先于 fromROSMsg 判断:避免 PCL 打印 "No data to copy"，
    // 同时保证下游每帧都能收到消息(空云也发)。
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      pub_->publish(*msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      pub_->publish(*msg);
      return;
    }

    // KdTree 作为近邻搜索结构, 供欧式聚类按距离扩张使用。
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(static_cast<float>(cluster_tolerance_));
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters);

    std::size_t max_cluster_points = 0;
    for (const auto & cluster : clusters) {
      max_cluster_points = std::max(max_cluster_points, cluster.indices.size());
    }

    if (log_cluster_centers_ && clusters.empty()) {
      RCLCPP_INFO(
        this->get_logger(), "frame %d.%09u [%s]: 0 clusters",
        static_cast<int>(msg->header.stamp.sec),
        static_cast<unsigned int>(msg->header.stamp.nanosec),
        msg->header.frame_id.c_str());
    }

    // 给每个聚类打上不同的 intensity 标签, 便于在 RViz 中按 intensity 着色区分。
    pcl::PointCloud<pcl::PointXYZI> labeled;
    labeled.reserve(cloud->size());
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      if (log_cluster_centers_) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, clusters[i].indices, centroid);
        RCLCPP_INFO(
          this->get_logger(),
          "frame %d.%09u [%s] cluster %zu: points=%zu center=(%.3f, %.3f, %.3f)",
          static_cast<int>(msg->header.stamp.sec),
          static_cast<unsigned int>(msg->header.stamp.nanosec),
          msg->header.frame_id.c_str(),
          i + 1, clusters[i].indices.size(),
          centroid[0], centroid[1], centroid[2]);
      }

      const float label = static_cast<float>(i + 1);
      for (const int idx : clusters[i].indices) {
        pcl::PointXYZI p = (*cloud)[idx];
        p.intensity = label;  // 用聚类编号覆盖强度, 作为可视化标签
        labeled.push_back(p);
      }
    }
    labeled.width = labeled.size();
    labeled.height = 1;
    labeled.is_dense = true;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(labeled, out);
    out.header = msg->header;
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "聚类: 输入 %zu 点 -> %zu 个簇, 最大簇 %zu 点, 输出 %zu 点",
      cloud->size(), clusters.size(), max_cluster_points, labeled.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  bool log_cluster_centers_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EuclideanClusterNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("euclidean_cluster"), "节点异常退出: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
