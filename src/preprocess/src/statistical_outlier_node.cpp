// 统计离群点去除节点 (背景过滤 / 去噪,预处理流水线的一环)
//
// 原理 (pcl::StatisticalOutlierRemoval):
//   对每个点,计算它到最近 mean_k 个邻居的平均距离;
//   统计所有点的平均距离,得到全局 均值 μ 和 标准差 σ;
//   若某点的平均邻居距离 > μ + stddev_mul_thresh * σ,
//   则判定为离群点(稀疏噪声/飞点)并剔除。
//
// 数据流:
//   input_topic (sensor_msgs/PointCloud2, PointXYZI)
//     -> StatisticalOutlierRemoval
//     -> 发布到 output_topic

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

class StatisticalOutlierNode : public rclcpp::Node
{
public:
  StatisticalOutlierNode()
  : Node("statistical_outlier")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/preprocess/valid");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/preprocess/output");

    // 用于估计平均距离的邻居点个数(越大越平滑,但更耗时)
    mean_k_ = this->declare_parameter<int>("mean_k", 20);
    // 标准差倍数阈值:μ + stddev_mul_thresh * σ 之外的点被剔除(越小越严格)
    stddev_mul_thresh_ = this->declare_parameter<double>("stddev_mul_thresh", 1.0);
    // negative=true 时反选:只保留被判定为离群的点(调试用)
    negative_ = this->declare_parameter<bool>("negative", false);

    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&StatisticalOutlierNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "统计离群点去除已启动: 订阅=%s 发布=%s | mean_k=%d stddev_mul_thresh=%.2f negative=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      mean_k_, stddev_mul_thresh_, negative_ ? "true" : "false");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI> filtered;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_mul_thresh_);
    sor.setNegative(negative_);
    sor.filter(filtered);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(filtered, out);
    out.header = msg->header;
    pub_->publish(out);

    RCLCPP_DEBUG(
      this->get_logger(), "输入点数: %zu -> 去离群后点数: %zu",
      cloud->size(), filtered.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  int mean_k_;
  double stddev_mul_thresh_;
  bool negative_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatisticalOutlierNode>());
  rclcpp::shutdown();
  return 0;
}
