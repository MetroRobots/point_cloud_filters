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

#include <three_d_laser_pc_filter/ring_filter.h>
#include <angles/angles.h>

namespace three_d_laser_pc_filter
{
// convert an angle in degrees to radians, and calculate the tan of that angle
inline float tanDeg(double angle_deg)
{
  double angle_rad = angles::from_degrees(angle_deg);
  return tanf(angle_rad);
}

bool RingFilter::configure()
{
  double min_angle_deg, max_angle_deg;
  getParam("min_angle_deg", min_angle_deg, false, 10.0);
  getParam("max_angle_deg", max_angle_deg, false, 170.0);

  min_angle_tan_ = tanDeg(min_angle_deg);
  max_angle_tan_ = tanDeg(max_angle_deg);

  int num_rings_signed;
  getParam("num_rings", num_rings_signed, true, 128);
  num_rings_ = num_rings_signed;
  RCLCPP_INFO(logging_interface_->get_logger(), "Ring Filter configured for %d rings", num_rings_);

  getParam("invert_filter", invert_filter_, false, false);  // for debugging
  return true;
}

rcl_interfaces::msg::SetParametersResult RingFilter::reconfigureCB(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (auto parameter : parameters)
  {
    RCLCPP_INFO(logging_interface_->get_logger(), "Update parameter %s to %s", parameter.get_name().c_str(),
                parameter.value_to_string().c_str());

    if (parameterMatches(parameter, "min_angle_deg", rclcpp::ParameterType::PARAMETER_DOUBLE))
    {
      min_angle_tan_ = tanDeg(parameter.as_double());
    }
    else if (parameterMatches(parameter, "max_angle_deg", rclcpp::ParameterType::PARAMETER_DOUBLE))
    {
      max_angle_tan_ = tanDeg(parameter.as_double());
    }
    else if (parameterMatches(parameter, "invert_filter", rclcpp::ParameterType::PARAMETER_BOOL))
    {
      invert_filter_ = parameter.as_bool();
    }
  }

  return result;
}

/**
 * Helper fne to get the addresses of the x y and z coordinates from a point with a specified ring and (point) index.
 */
inline void getPt(void* data_start, const unsigned int ring_i, const unsigned int num_rings, const unsigned int point_i,
                  unsigned int point_step, float*& x, float*& y, float*& z)
{
  // these should be dynamic but for speed are not
  const static int y_offset = 1;
  const static int z_offset = 2;

  size_t base_index = point_i * num_rings + ring_i;
  size_t index = base_index * point_step;
  void* new_p = data_start + index;
  x = (float*)new_p;
  y = (float*)new_p + y_offset;
  z = (float*)new_p + z_offset;
}

bool RingFilter::update(const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out)
{
  data_out = data_in;

  unsigned int N = data_out.width * data_out.height;
  unsigned int num_points = N / num_rings_;
  unsigned int filtered_points_count = 0;
  void* data_start = (void*)&(data_out.data.front());

  // The addresses of the previous and current points under consideration
  float *pt0_x, *pt0_y, *pt0_z, *pt1_x, *pt1_y, *pt1_z;
  // The values of the points under consideration
  float x0, y0, z0, x1, y1, z1;
  for (unsigned int ring_i = 0; ring_i < num_rings_; ring_i++)
  {
    getPt(data_start, ring_i, num_rings_, 0, data_out.point_step, pt0_x, pt0_y, pt0_z);
    x0 = *pt0_x;
    y0 = *pt0_y;
    z0 = *pt0_z;
    // cache the distance to the previous point
    float r0 = sqrtf(x0 * x0 + y0 * y0 + z0 * z0);
    for (unsigned int point_i = 1; point_i < num_points; point_i++)
    {
      getPt(data_start, ring_i, num_rings_, point_i, data_out.point_step, pt1_x, pt1_y, pt1_z);
      x1 = *pt1_x;
      y1 = *pt1_y;
      z1 = *pt1_z;
      // get the distance to the current point
      float r1 = sqrtf(x1 * x1 + y1 * y1 + z1 * z1);

      if (isShadow(r0, r1, x0, y0, z0, x1, y1, z1) == !invert_filter_)
      {
        *pt0_x = std::numeric_limits<float>::quiet_NaN();
        filtered_points_count++;
      }

      // update the cached values
      pt0_x = pt1_x;
      pt0_y = pt1_y;
      pt0_z = pt1_z;
      x0 = x1;
      y0 = y1;
      z0 = z1;
      r0 = r1;
    }
  }

  RCLCPP_DEBUG(logging_interface_->get_logger(), "Ring Filter %d/%d points", filtered_points_count, N);

  return true;
}

}  // namespace three_d_laser_pc_filter

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(three_d_laser_pc_filter::RingFilter, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
