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

#include <point_cloud_filters/cloud_filter_chain.hpp>

namespace point_cloud_filters
{
CloudFilterChain::CloudFilterChain(const rclcpp::NodeOptions& options, const std::string& ns)
  : rclcpp::Node("cloud_filter_chain", ns, options), tf_(NULL), buffer_(get_clock()), tf_filter_(NULL),
    filter_chain_("sensor_msgs::msg::PointCloud2")
{
  filter_chain_.configure("", get_node_logging_interface(), get_node_parameters_interface());

  rcl_interfaces::msg::ParameterDescriptor read_only_desc;
  read_only_desc.read_only = true;

  declare_parameter("in_topic", "cloud", read_only_desc);
  declare_parameter("out_topic", "cloud_filtered", read_only_desc);
  declare_parameter("lazy_subscription", false, read_only_desc);
  declare_parameter("target_frame", "", read_only_desc);
  declare_parameter("tf_tolerance", 0.03, read_only_desc);
  declare_parameter("publisher_history_depth", 1000);
  declare_parameter("publish_frequency", -1.0);

  // Get parameters
  get_parameter("in_topic", in_topic_);
  get_parameter("out_topic", out_topic_);
  get_parameter("lazy_subscription", lazy_subscription_);
  get_parameter("target_frame", target_frame_);
  get_parameter("tf_tolerance", tf_tolerance_);
  get_parameter("publisher_history_depth", publisher_history_depth_);

  RCLCPP_INFO(get_logger(), "Filtering cloud from %s to %s", in_topic_.c_str(), out_topic_.c_str());

  if (!target_frame_.empty())
  {
    tf_.reset(new tf2_ros::TransformListener(buffer_));
    tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
        cloud_sub_, buffer_, "", 50, get_node_logging_interface(), get_node_clock_interface()));
    tf_filter_->setTargetFrame(target_frame_);
    tf_filter_->setTolerance(std::chrono::duration<double>(tf_tolerance_));

    // Setup tf::MessageFilter generates callback
    tf_filter_->registerCallback(std::bind(&CloudFilterChain::callback, this, std::placeholders::_1));
  }
  else
  {
    // Pass through if no tf_message_filter_target_frame
    cloud_sub_.registerCallback(std::bind(&CloudFilterChain::callback, this, std::placeholders::_1));
  }

  if (lazy_subscription_)
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo& s) {
      if (s.current_count == 0)
      {
        cloud_sub_.unsubscribe();
      }
      else if (!cloud_sub_.getSubscriber())
      {
        cloud_sub_.subscribe(this, in_topic_, rmw_qos_profile_sensor_data);
      }
    };
    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, publisher_history_depth_, pub_options);
  }
  else
  {
    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, publisher_history_depth_);
    cloud_sub_.subscribe(this, in_topic_, rmw_qos_profile_sensor_data);
  }
}

CloudFilterChain::~CloudFilterChain()
{
  if (tf_filter_)
  {
    tf_filter_.reset();
  }
  if (tf_)
  {
    tf_.reset();
  }
}

void CloudFilterChain::callback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg_in)
{
  // Throttling check
  double freq;
  get_parameter("publish_frequency", freq);
  rclcpp::Time tnow = now();

  if (freq == 0.0)
  {
    return;
  }
  else if (published_ && freq > 0.0)
  {
    rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / freq);
    if (tnow - last_pub_ < period)
    {
      return;
    }
  }
  last_pub_ = tnow;
  published_ = true;

  sensor_msgs::msg::PointCloud2 msg_out;
  // Run the filter chain
  if (filter_chain_.update(*msg_in, msg_out))
  {
    //only publish result if filter succeeded
    output_pub_->publish(msg_out);
  }
}

}  // namespace point_cloud_filters

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_filters::CloudFilterChain)
