// ROI 点云裁剪节点
//
// 数据流:
//   /livox/lidar (sensor_msgs/PointCloud2, Livox PointXYZRTL, xfer_format=0)
//        -> pcl::fromROSMsg 转为 pcl::PointXYZI (按字段名映射 x/y/z/intensity)
//        -> pcl::CropBox 按可配置的 x/y/z 边界(单位:米)裁剪
//        -> 发布标准 sensor_msgs/PointCloud2 (PointXYZI) 到 /livox/lidar/roi
//
// 说明:MID-70 必须使用仓库自带的 livox_ros2_driver(基于 Livox-SDK)。
//       该驱动版本已禁用 xfer_format=2(PointXYZI)分支,因此这里用 xfer_format=0
//       作为输入,输出仍为标准 PointXYZI 点云,效果等价。

#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class RoiFilterNode : public rclcpp::Node
{
public:
  RoiFilterNode()
  : Node("roi_filter")
  {
    // ---- 可配置参数(边界单位均为:米)----
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar/roi");

    x_min_ = this->declare_parameter<double>("x_min", -10.0);
    x_max_ = this->declare_parameter<double>("x_max",  10.0);
    y_min_ = this->declare_parameter<double>("y_min", -10.0);
    y_max_ = this->declare_parameter<double>("y_max",  10.0);
    z_min_ = this->declare_parameter<double>("z_min",  -2.0);
    z_max_ = this->declare_parameter<double>("z_max",   2.0);

    // negative=true 时保留 ROI 盒子“之外”的点(反选)
    negative_ = this->declare_parameter<bool>("negative", false);

    const rclcpp::QoS qos = rclcpp::QoS(10);  // 与 livox 驱动默认的 reliable QoS 匹配

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&RoiFilterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "ROI filter 已启动: 订阅=%s 发布=%s | x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f] (m) negative=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
      negative_ ? "true" : "false");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // ROS -> PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    // CropBox 裁剪(非有限点在边界比较中为 false,会被自动剔除)
    pcl::PointCloud<pcl::PointXYZI> roi;
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(
      static_cast<float>(x_min_), static_cast<float>(y_min_), static_cast<float>(z_min_), 1.0f));
    crop.setMax(Eigen::Vector4f(
      static_cast<float>(x_max_), static_cast<float>(y_max_), static_cast<float>(z_max_), 1.0f));
    crop.setNegative(negative_);
    crop.filter(roi);

    // PCL -> ROS,并沿用输入的 header(时间戳/frame_id)
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(roi, out);
    out.header = msg->header;
    pub_->publish(out);

    RCLCPP_DEBUG(
      this->get_logger(), "原始点数: %zu -> ROI 点数: %zu",
      cloud->size(), roi.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  bool negative_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiFilterNode>());
  rclcpp::shutdown();
  return 0;
}
