// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "string2nav2/string2nav2_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cstdlib>

using namespace std::chrono_literals;

namespace string2nav2 {

String2Nav2::String2Nav2() : Node("string2nav2_node") { initPubSub(); }

void String2Nav2::initPubSub() {
  pub_speak_ = this->create_publisher<std_msgs::msg::String>("speak", 1);

  sub_voice2string_ = create_subscription<std_msgs::msg::String>(
      "voice2string", 1,
      std::bind(&String2Nav2::voice2stringCb, this, std::placeholders::_1));
}

void String2Nav2::voice2stringCb(
    const std_msgs::msg::String::ConstSharedPtr msg) {

  if (msg->data.find("シミュレータ") != std::string::npos ||
      msg->data.find("しみゅれーた") != std::string::npos) {
    std::system("rviz2 &");
    auto speak_str = std_msgs::msg::String();
    speak_str.data = "　シミュレータを立ち上げるで！";
    pub_speak_->publish(speak_str);
  }
}

} // namespace string2nav2
