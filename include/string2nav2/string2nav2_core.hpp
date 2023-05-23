// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_
#define ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
using Client = rclcpp_action::Client<NavigateToPoseAction>;

namespace string2nav2 {

class String2Nav2 : public rclcpp::Node {
public:
  String2Nav2();

protected:
  void initPubSub();
  void initClient();

  void voice2stringCb(const std_msgs::msg::String::ConstSharedPtr msg);

  void send_goal(const double x, const double y, const double theta);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speak_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_initpose_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_voice2string_;
  rclcpp_action::Client<NavigateToPoseAction>::SharedPtr client_nav2_;
};

} // namespace string2nav2

#endif // ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_
