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

#pragma once

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace three_d_laser_pc_filter
{
/**
 * Filters shadows based on adjacent points in concentric rings
 *
 * Say there are N rings (parameter num_rings) and M points in each ring.
 *
 * Makes strong assumptions for speed that data is formatted in an index-major order, i.e.
 * Point 0         | Ring 0     Index 0
 * Point 1         | Ring 1     Index 0
 * ...
 * Point N - 1     | Ring N - 1 Index 0
 * Point N         | Ring 0     Index 1
 * Point N + 1     | Ring 1     Index 1
 * ...
 * Point N * M - 1 | Ring N - 1 Index M - 1
 *
 * Consider two adjacent points on Ring i, with consecutive indexes j and j + 1
 * which we call p0 and p1. We filter based on the same approach used by standard
 * 2d laser filters, i.e. calculating the angle formed by the origin, p0 and p1 (in that order)
 * and cutting off angles exceeding parametric minimum and maximum angles
 * (i.e. the parameters min_angle_deg and max_angle_deg)
 */
class RingFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
{
public:
  bool configure() override;
  rcl_interfaces::msg::SetParametersResult reconfigureCB(std::vector<rclcpp::Parameter> parameters) override;
  bool update(const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out) override;

protected:
  // generic helper method that I should merge in upstream
  bool parameterMatches(const rclcpp::Parameter& parameter, const std::string& param_name,
                        const rclcpp::ParameterType& param_type)
  {
    std::string target_name = param_prefix_ + param_name;
    return parameter.get_name() == target_name && parameter.get_type() == param_type;
  }

  /**
   * Determines if the given points result in a shadow or not.
   *
   * Note: r0 and r1 are redundant here, given the points, but are passed in manually for caching purposes
   */
  inline bool isShadow(float r0, float r1, float x0, float y0, float z0, float x1, float y1, float z1)
  {
    // Two references:
    // 1) https://math.stackexchange.com/questions/2521886/how-to-find-angle-between-2-points-in-3d-space
    // 2) https://johntgz.github.io/2022/01/09/the_ultimate_guide_to_laser_filters/#scanshadowsfilter

    // To calculate the angle between the two points, we typically would solve for alpha in
    // cos(alpha) = (v0 dot v1) / (r0 * r1)
    float dot = x0 * x1 + y0 * y1 + z0 * z1;
    float cos_alpha = dot / (r0 * r1);

    // Then to calculate perpendicular_x and perpendicular_y (from reference 2)
    // which requires cos(alpha) and sin(alpha).
    // Rather than calculate alpha with acos, we instead simply calculate sin(alpha) from cos_alpha
    float sin_alpha = sqrtf(-cos_alpha * cos_alpha + 1);

    float perpendicular_x = r0 - r1 * cos_alpha;
    float perpendicular_y = r1 * sin_alpha;
    float tan_theta = abs(perpendicular_y) / perpendicular_x;

    // Plus, instead of computing theta using atan, we simply compare against the precomputed tans of the limits
    if (tan_theta > 0)
    {
      if (tan_theta < min_angle_tan_)
        return true;
    }
    else
    {
      if (tan_theta > max_angle_tan_)
        return true;
    }
    return false;
  }

  unsigned int num_rings_;
  double min_angle_tan_, max_angle_tan_;
  bool invert_filter_;
};
}  // namespace three_d_laser_pc_filter
