#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "pointcloud_preprocessing/field_filter.hpp"

namespace pointcloud_preprocessing
{
class FieldFilterNode : public rclcpp::Node
{
public:
  FieldFilterNode() : Node("field_filter_node")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto param = [&](const std::string & name, auto value) {
        return declare_parameter(name, value, descriptor);
      };
    FieldConfig config;
    config.length = param("field.length_m", 10.0);
    config.width = param("field.width_m", 10.0);
    config.min_x = param("field.min_x_m", 0.0);
    config.min_y = param("field.min_y_m", -5.0);
    config.roll_deg = param("mounting.roll_deg", 0.0);
    config.pitch_deg = param("mounting.pitch_deg", 0.0);
    config.yaw_deg = param("mounting.yaw_deg", 0.0);
    filter_ = std::make_unique<FieldFilter>(config);
    const auto input = param("input_topic", std::string("/livox/lidar"));
    const auto output = param("output_topic", std::string("/radar/points_filtered"));
    expected_frame_ = param("input_frame", std::string("livox_frame"));
    if (expected_frame_.empty()) {throw std::invalid_argument("input_frame must not be empty");}
    if (get_node_topics_interface()->resolve_topic_name(input) ==
      get_node_topics_interface()->resolve_topic_name(output)) {
      throw std::invalid_argument("Input and output topics must differ");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        try {
          if (message->header.frame_id != expected_frame_) {
            throw std::invalid_argument("Unexpected input frame: " + message->header.frame_id);
          }
          publisher_->publish(filter_->filter(*message));
        } catch (const std::exception & error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
        }
      });
    RCLCPP_INFO(get_logger(), "Field ROI x=[%.3f, %.3f], y=[%.3f, %.3f] m; RPY=[%.2f, %.2f, %.2f] deg",
      config.min_x, config.min_x + config.length, config.min_y, config.min_y + config.width,
      config.roll_deg, config.pitch_deg, config.yaw_deg);
  }
private:
  std::unique_ptr<FieldFilter> filter_;
  std::string expected_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace pointcloud_preprocessing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pointcloud_preprocessing::FieldFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("field_filter_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
