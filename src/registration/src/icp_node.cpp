#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class IcpNode : public rclcpp::Node
{
public:
  IcpNode()
  : Node("icp")
  {
    // 声明参数
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/preprocess/valid");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/registration/icp");
    target_topic_ = this->declare_parameter<std::string>("target_topic", "/registration/target");
    target_file_ = this->declare_parameter<std::string>("target_file", "");

    max_iterations_ = this->declare_parameter<int>("max_iterations", 50);
    max_correspondence_distance_ = this->declare_parameter<double>("max_correspondence_distance", 0.5);
    target_leaf_size_ = this->declare_parameter<double>("target_leaf_size", 0.1);
    source_leaf_size_ = this->declare_parameter<double>("source_leaf_size", 0.1);
    fitness_threshold_ = this->declare_parameter<double>("fitness_threshold", 0.05);

    // 加载PCD文件
    loadTarget(target_file_);

    if (!target_cloud_ || target_cloud_ -> empty()){
      RCLCPP_ERROR(this->get_logger(), "参考点云为空，节点无法工作！请检查 target_file 参数");
      throw std::runtime_error("target cloud is empty");
    }

    // 目标点云只设置一次,KD 树只构建一次,避免每帧重建
    icp_.setInputTarget(target_cloud_);
    icp_.setMaximumIterations(max_iterations_);
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_);

    // 初始化上一帧为单位矩阵
    prev_transform_ = Eigen::Matrix4f::Identity();

    // 创建发布者和订阅者
    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    // 参考点云只发一次,transient_local 保证后启动的 RViz 也能收到
    target_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      target_topic_, rclcpp::QoS(1).transient_local());
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&IcpNode::cloudCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "ICP 已启动: 订阅=%s 发布=%s 参考点云话题=%s",
      input_topic_.c_str(), output_topic_.c_str(), target_topic_.c_str());
  }

  private:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & in, double leaf)
    {
      if (leaf <= 0.0) return in;
      pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(in);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*out);
      return out;
    }

    void loadTarget(const std::string & path){
      pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());

      if (path.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(path, *raw) < 0){
        RCLCPP_ERROR(this->get_logger(), "PCD 文件加载失败: '%s'", path.c_str());
        return;
      }

      target_cloud_ = downsample(raw, target_leaf_size_);
      RCLCPP_INFO(this->get_logger(), "参考点云加载完成: %zu 个点 -> 降采样后 %zu 个点",
        raw->size(), target_cloud_->size());
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
      if (!target_published_) {
        sensor_msgs::msg::PointCloud2 target_msg;
        pcl::toROSMsg(*target_cloud_, target_msg);
        target_msg.header = msg->header;
        target_pub_->publish(target_msg);
        target_published_ = true;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr source_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *source_xyzi);

      if (source_xyzi->empty()) return;

      pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*source_xyzi, *source_xyz);
      source_xyz = downsample(source_xyz, source_leaf_size_);

      icp_.setInputSource(source_xyz);   //实时点云输入

      pcl::PointCloud<pcl::PointXYZ> aligned;
      icp_.align(aligned, prev_transform_);

      const double score = icp_.getFitnessScore();
      if (!icp_.hasConverged() || score > fitness_threshold_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "ICP 配准失败 | converged=%d fitness_score=%.6f (阈值 %.6f) 源点数=%zu，重置初值",
          icp_.hasConverged(), score, fitness_threshold_, source_xyz->size());
        prev_transform_ = Eigen::Matrix4f::Identity();
        return;
      }

      prev_transform_ = icp_.getFinalTransformation();

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "ICP 收敛 | fitness_score=%.6f 源点数=%zu", score, source_xyz->size());

      pcl::PointCloud<pcl::PointXYZI> transformed;
      pcl::transformPointCloud(*source_xyzi, transformed, prev_transform_);

      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(transformed, out);
      out.header = msg->header;
      pub_->publish(out);
    }

    std::string input_topic_, output_topic_, target_topic_, target_file_;
    int max_iterations_;
    double max_correspondence_distance_;
    double target_leaf_size_, source_leaf_size_;
    double fitness_threshold_;
    bool target_published_ = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    Eigen::Matrix4f prev_transform_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpNode>());
  rclcpp::shutdown();
  return 0;
}
