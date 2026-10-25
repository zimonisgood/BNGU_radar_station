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

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&EuclideanClusterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "欧式聚类已启动: 订阅=%s 发布=%s | tolerance=%.3fm size=[%d, %d]",
      input_topic_.c_str(), output_topic_.c_str(),
      cluster_tolerance_, min_cluster_size_, max_cluster_size_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
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

    // 给每个聚类打上不同的 intensity 标签, 便于在 RViz 中按 intensity 着色区分。
    pcl::PointCloud<pcl::PointXYZI> labeled;
    labeled.reserve(cloud->size());
    for (std::size_t i = 0; i < clusters.size(); ++i) {
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

    RCLCPP_DEBUG(
      this->get_logger(), "输入点数: %zu -> 聚类数: %zu (聚类内点数: %zu)",
      cloud->size(), clusters.size(), labeled.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EuclideanClusterNode>());
  rclcpp::shutdown();
  return 0;
}
