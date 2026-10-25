#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class IcpNode : public rclcpp::Node
{
public:
  IcpNode()
  : Node("icp")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/preprocess/valid");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/registration/icp");

    // 将预处理发布的valid ROS2消息转为pcd点云消息
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
      pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *source);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpNode>());
  rclcpp::shutdown();
  return 0;
}