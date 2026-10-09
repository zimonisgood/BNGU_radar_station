// 无效点去除节点 (预处理流水线的一环)
//
// 功能:去除点云中的非有限点 (NaN / Inf)。
//   - Livox 原始点云中可能存在无效测量 (NaN/Inf),若不剔除,
//     后续的统计/半径离群点去除、地面拟合等算法会得到错误结果或报错。
//
// 数据流:
//   input_topic (sensor_msgs/PointCloud2, PointXYZI)
//     -> pcl::removeNaNFromPointCloud 去除非有限点
//     -> 发布到 output_topic (稠密点云 is_dense=true)

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>  // removeNaNFromPointCloud
#include <pcl_conversions/pcl_conversions.h>

class InvalidRemovalNode : public rclcpp::Node
{
public:
  InvalidRemovalNode()
  : Node("invalid_removal")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/preprocess/roi");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/preprocess/valid");

    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&InvalidRemovalNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "无效点去除已启动: 订阅=%s 发布=%s (去除 NaN/Inf)",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI> valid;
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, valid, index);
    valid.is_dense = true;  // 去除无效点后标记为稠密

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(valid, out);
    out.header = msg->header;
    pub_->publish(out);

    RCLCPP_DEBUG(
      this->get_logger(), "输入点数: %zu -> 有效点数: %zu",
      cloud->size(), valid.size());
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InvalidRemovalNode>());
  rclcpp::shutdown();
  return 0;
}
