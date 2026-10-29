#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
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
    // stl_file_ = this->declare_parameter<std::string>("stl_file", "");
    target_file_ = this->declare_parameter<std::string>("target", "");

    max_iterations_ = this->declare_parameter<int>("max_iterations", 50);
    max_correspondence_distance_ = this->declare_parameter<double>("max_correspondence_distance", 0.5);

    // 加载STL文件
    // loadTarget(stl_file_);
    // 加载PCD文件
    loadPCDFile(target_file_);

    // 初始化上一帧为单位矩阵
    prev_transform_ = Eigen::Matrix4f::Identity();

    // 创建发布者和订阅者
    const rclcpp::QoS qos = rclcpp::QoS(10);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&IcpNode::cloudCallback, this, std::placeholders::_1)
    );
  }

  private:
    // void loadTarget(const std::string & path){
    //   pcl::PolygonMesh mesh;
    //   if (pcl::io::loadPolygonFileSTL(path, mesh) < 0){
    //     RCLCPP_ERROR(this->get_logger(), "STL 文件加载失败： %s", path.c_str());
    //     return;
    //   }

    //   target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    //   pcl::fromPCLPointCloud2(mesh.cloud, *target_cloud_);

    //   RCLCPP_INFO(this->get_logger(), "STL 加载完成： %zu 个顶点", target_cloud_->size());
    // }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
      pcl::PointCloud<pcl::PointXYZI>::Ptr source_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *source_xyzi);

      if (source_xyzi->empty()) return;

      pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*source_xyzi, *source_xyz);

      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(source_xyz);   //实时点云输入
      icp.setInputTarget(target_cloud_);
      icp.setMaximumIterations(max_iterations_);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance_);

      pcl::PointCloud<pcl::PointXYZ> aligned;
      icp.align(aligned, prev_transform_);

      if (!icp.hasConverged()){
        RCLCPP_WARN(this->get_logger(), "ICP 未收敛，跳过该帧");
        return;
      }

      if (icp.hasConverged()){
        printf("it has been converged.");
        return;
      }

      prev_transform_ = icp.getFinalTransformation();

      pcl::PointCloud<pcl::PointXYZI> transformed;
      pcl::transformPointCloud(*source_xyzi, transformed, prev_transform_);

      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(transformed, out);
      out.header = msg->header;
      pub_->publish(out);
    }

    std::string input_topic_, output_topic_, stl_file_;
    int max_iterations_;
    double max_correspondence_distance_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    Eigen::Matrix4f prev_transform_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpNode>());
  rclcpp::shutdown();
  return 0;
}