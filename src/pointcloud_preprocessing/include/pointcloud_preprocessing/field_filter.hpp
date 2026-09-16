#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

namespace pointcloud_preprocessing
{
struct FieldConfig
{
  double length{10.0}, width{10.0}, min_x{0.0}, min_y{-5.0};
  double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
};

class FieldFilter
{
public:
  explicit FieldFilter(const FieldConfig & config) : config_(config)
  {
    for (double value : {config.length, config.width, config.min_x, config.min_y,
        config.roll_deg, config.pitch_deg, config.yaw_deg,
        config.min_x + config.length, config.min_y + config.width})
    {
      if (!std::isfinite(value)) {throw std::invalid_argument("Field parameters must be finite");}
    }
    if (config.length <= 0 || config.width <= 0) {
      throw std::invalid_argument("Field length and width must be positive");
    }
    constexpr double rad = 3.14159265358979323846 / 180.0;
    rotation_.setRPY(config.roll_deg * rad, config.pitch_deg * rad, config.yaw_deg * rad);
  }

  sensor_msgs::msg::PointCloud2 filter(const sensor_msgs::msg::PointCloud2 & input) const
  {
    using sensor_msgs::msg::PointField;
    std::array<uint32_t, 3> offsets{};
    const std::array<std::string, 3> names{"x", "y", "z"};
    for (size_t axis = 0; axis < names.size(); ++axis) {
      auto field = std::find_if(input.fields.begin(), input.fields.end(),
        [&](const auto & f) {return f.name == names[axis];});
      if (field == input.fields.end() || field->datatype != PointField::FLOAT32 ||
        field->count != 1 || uint64_t(field->offset) + sizeof(float) > input.point_step)
      {
        throw std::invalid_argument("Expected valid scalar FLOAT32 x/y/z fields");
      }
      offsets[axis] = field->offset;
    }
    const uint64_t packed_row = uint64_t(input.width) * input.point_step;
    if (input.row_step < packed_row ||
      uint64_t(input.row_step) * input.height > input.data.size())
    {
      throw std::invalid_argument("Malformed PointCloud2 row_step or data size");
    }
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.height = 1;
    output.is_dense = true;
    output.data.reserve(input.data.size());
    for (uint32_t row = 0; row < input.height; ++row) {
      for (uint32_t col = 0; col < input.width; ++col) {
        const auto * point = input.data.data() + size_t(row) * input.row_step +
          size_t(col) * input.point_step;
        const float x = readFloat(point + offsets[0], input.is_bigendian);
        const float y = readFloat(point + offsets[1], input.is_bigendian);
        const float z = readFloat(point + offsets[2], input.is_bigendian);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
        const auto p = rotation_ * tf2::Vector3(x, y, z);
        if (p.x() < config_.min_x || p.x() > config_.min_x + config_.length ||
          p.y() < config_.min_y || p.y() > config_.min_y + config_.width) {continue;}
        if (output.data.size() + input.point_step > std::numeric_limits<uint32_t>::max()) {
          throw std::invalid_argument("Filtered cloud exceeds PointCloud2 row_step capacity");
        }
        output.data.insert(output.data.end(), point, point + input.point_step);
        ++output.width;
      }
    }
    output.row_step = static_cast<uint32_t>(output.data.size());
    return output;
  }

private:
  static float readFloat(const uint8_t * data, bool big_endian)
  {
    uint32_t bits = 0;
    for (unsigned i = 0; i < 4; ++i) {
      bits |= uint32_t(data[i]) << (8 * (big_endian ? 3 - i : i));
    }
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  FieldConfig config_;
  tf2::Matrix3x3 rotation_;
};
}  // namespace pointcloud_preprocessing
