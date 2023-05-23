// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_
#define ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace string2nav2 {

class String2Nav2 : public rclcpp::Node {
public:
  String2Nav2();

protected:
  void initPubSub();

  void voice2stringCb(const std_msgs::msg::String::ConstSharedPtr msg);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_speak_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_voice2string_;
};

} // namespace string2nav2

#endif // ROS2_CPP_TEMPLATE__NODE__TEMPLATE_CORE_HPP_
