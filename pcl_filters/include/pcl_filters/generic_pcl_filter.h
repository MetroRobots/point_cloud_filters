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
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_filters
{
/**
 * Generic PCL Filter enables running PCL operations on PointCloud2 messages by
 * first converting to a pcl::PCLPointCloud2 and then running the abstract update function
 * and converting the result back into a PointCloud2 message.
 */
class GenericPCLFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
{
public:
  bool update(const sensor_msgs::msg::PointCloud2& data_in, sensor_msgs::msg::PointCloud2& data_out) override
  {
    pcl::PCLPointCloud2::Ptr cloud = std::make_shared<pcl::PCLPointCloud2>();
    pcl::PCLPointCloud2::Ptr cloud_filtered = std::make_shared<pcl::PCLPointCloud2>();

    pcl_conversions::toPCL(data_in, *cloud);

    bool ret = update(cloud, cloud_filtered);
    if (!ret)
      return ret;

    pcl_conversions::fromPCL(*cloud_filtered, data_out);
    data_out.header.frame_id = data_in.header.frame_id;
    data_out.header.stamp = data_in.header.stamp;

    return ret;
  }

  virtual bool update(const pcl::PCLPointCloud2::ConstPtr& cloud_in, const pcl::PCLPointCloud2::Ptr& cloud_out) = 0;
};
}  // namespace pcl_filters
