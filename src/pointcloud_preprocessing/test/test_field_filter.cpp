#include <gtest/gtest.h>
#include "pointcloud_preprocessing/field_filter.hpp"

using pointcloud_preprocessing::FieldConfig;
using pointcloud_preprocessing::FieldFilter;
using sensor_msgs::msg::PointCloud2;

PointCloud2 cloud(const std::vector<std::array<float, 3>> & points, bool big = false)
{
  PointCloud2 msg;
  msg.header.frame_id = "livox_frame";
  msg.header.stamp.sec = 123;
  msg.height = 1;
  msg.width = points.size();
  msg.point_step = 18;
  msg.row_step = msg.width * msg.point_step;
  msg.is_bigendian = big;
  for (unsigned i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = std::array<std::string, 3>{"x", "y", "z"}[i];
    f.offset = i * 4;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.data.resize(msg.row_step, 0x5a);  // Extra Livox fields must survive byte-for-byte.
  for (size_t p = 0; p < points.size(); ++p) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      uint32_t bits;
      std::memcpy(&bits, &points[p][axis], 4);
      for (unsigned b = 0; b < 4; ++b) {
        msg.data[p * 18 + axis * 4 + b] = (bits >> (8 * (big ? 3 - b : b))) & 255;
      }
    }
  }
  return msg;
}

TEST(FieldFilter, InclusiveBoundsNonfiniteAndPayload)
{
  const auto input = cloud({{0, -5, -100}, {10, 5, 100}, {10.1f, 0, 0},
    {-0.1f, 0, 0}, {1, 5.1f, 0}, {1, 0, NAN}, {INFINITY, 0, 0}});
  const auto result = FieldFilter(FieldConfig{}).filter(input);
  EXPECT_EQ(result.width, 2u);
  EXPECT_EQ(result.row_step, 36u);
  EXPECT_EQ(result.header, input.header);
  EXPECT_EQ(result.fields, input.fields);
  EXPECT_EQ(result.data, std::vector<uint8_t>(input.data.begin(), input.data.begin() + 36));
  EXPECT_TRUE(result.is_dense);
}

TEST(FieldFilter, InvertedTiltAndYaw)
{
  FieldConfig config;
  config.roll_deg = 180;
  config.pitch_deg = 30;
  config.yaw_deg = 90;
  config.min_x = 1.5;
  config.min_y = 0.5;
  config.length = 1;
  config.width = 1;
  // Ry(30)*Rx(180)*(sqrt(3), 2, 1)=(1,-2,-sqrt(3)); Rz(90) -> (2,1,-sqrt(3)).
  const auto input = cloud({{float(std::sqrt(3.0)), 2, 1}, {0, 0, 0}});
  const auto result = FieldFilter(config).filter(input);
  ASSERT_EQ(result.width, 1u);
  EXPECT_EQ(result.data, std::vector<uint8_t>(input.data.begin(), input.data.begin() + 18));
}

TEST(FieldFilter, BigEndianAndOrganizedRowPadding)
{
  auto input = cloud({{1, 0, 0}, {20, 0, 0}}, true);
  input.width = 1;
  input.height = 2;
  input.row_step = 22;
  input.data.insert(input.data.begin() + 18, 4, 0xff);
  input.data.insert(input.data.end(), 4, 0xff);
  const auto result = FieldFilter(FieldConfig{}).filter(input);
  EXPECT_EQ(result.width, 1u);
  EXPECT_EQ(result.height, 1u);
  EXPECT_TRUE(result.is_bigendian);
  EXPECT_EQ(result.data, std::vector<uint8_t>(input.data.begin(), input.data.begin() + 18));
}

TEST(FieldFilter, EmptyAndAllOutside)
{
  const FieldFilter filter(FieldConfig{});
  EXPECT_EQ(filter.filter(cloud({})).width, 0u);
  const auto result = filter.filter(cloud({{100, 0, 0}}));
  EXPECT_TRUE(result.data.empty());
  EXPECT_EQ(result.row_step, 0u);
}

TEST(FieldFilter, RejectInvalidConfigAndMessages)
{
  FieldConfig config;
  config.length = -1;
  EXPECT_THROW(FieldFilter{config}, std::invalid_argument);
  config.length = 10;
  config.roll_deg = NAN;
  EXPECT_THROW(FieldFilter{config}, std::invalid_argument);
  const FieldFilter filter(FieldConfig{});
  auto input = cloud({{1, 0, 0}});
  input.data.pop_back();
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1, 0, 0}});
  input.fields[0].offset = 17;
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1, 0, 0}});
  input.fields.pop_back();
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
}
