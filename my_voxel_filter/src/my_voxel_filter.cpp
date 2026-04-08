/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <my_voxel_filter/my_voxel_filter.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace my_voxel_filter
{
bool MyVoxelFilter::configure()
{
  double leaf_size_d;
  getParam("leaf_size", leaf_size_d, false, 0.05);
  leaf_size_ = leaf_size_d;
  inverse_leaf_size_ = 1.0 / leaf_size_;
  return true;
}

rcl_interfaces::msg::SetParametersResult MyVoxelFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto parameter : parameters)
  {
    RCLCPP_INFO(logging_interface_->get_logger(), "Update parameter %s to %s", parameter.get_name().c_str(),
                parameter.value_to_string().c_str());

    if (parameter.get_name() == param_prefix_ + "leaf_size")
    {
      leaf_size_ = parameter.as_double();
      inverse_leaf_size_ = 1.0 / leaf_size_;
    }
  }

  return result;
}

bool MyVoxelFilter::update(const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out)
{
  float min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = min_y = min_z = std::numeric_limits<float>::infinity();
  max_x = max_y = max_z = -std::numeric_limits<float>::infinity();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(data_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(data_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(data_in, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    min_x = std::min(min_x, *iter_x);
    min_y = std::min(min_y, *iter_y);
    min_z = std::min(min_z, *iter_z);
    max_x = std::max(max_x, *iter_x);
    max_y = std::max(max_y, *iter_y);
    max_z = std::max(max_z, *iter_z);
  }

  std::int64_t dx = static_cast<std::int64_t>((max_x - min_x) * inverse_leaf_size_) + 1;
  std::int64_t dy = static_cast<std::int64_t>((max_y - min_y) * inverse_leaf_size_) + 1;
  std::int64_t dz = static_cast<std::int64_t>((max_z - min_z) * inverse_leaf_size_) + 1;

  if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
  {
    RCLCPP_WARN(logging_interface_->get_logger(),
                "Leaf size is too small for the input dataset. Integer indices would overflow.");
    return false;
  }

  std::map<unsigned int, unsigned int> indexes;
  unsigned int count = 0;
  unsigned int index = 0;
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index)
  {
    unsigned int xi = (*iter_x - min_x) * inverse_leaf_size_;
    unsigned int yi = (*iter_y - min_y) * inverse_leaf_size_;
    unsigned int zi = (*iter_z - min_z) * inverse_leaf_size_;

    unsigned int code = (xi + dx * yi) * dy + zi;
    auto it = indexes.find(code);
    if (it == indexes.end())
    {
      indexes[code] = index;
      count += 1;
    }
  }

  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(data_in, "z");

  data_out.header = data_in.header;
  data_out.width = count;
  data_out.height = 1;
  data_out.fields = data_in.fields;
  data_out.is_bigendian = data_in.is_bigendian;
  data_out.point_step = data_in.point_step;
  data_out.row_step = data_in.point_step * count;
  data_out.is_dense = true;

  data_out.data.resize(data_in.point_step * count);

  index = 0;

  for (auto it = indexes.begin(); it != indexes.end(); it++, ++index)
  {
    std::memcpy(&data_out.data.front() + index * data_in.point_step,
                &data_in.data.front() + it->second * data_in.point_step, data_in.point_step);
  }

  return true;
}

}  // namespace my_voxel_filter

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_voxel_filter::MyVoxelFilter, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
