// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "string2nav2/string2nav2_core.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <cstdlib>

namespace string2nav2 {

String2Nav2::String2Nav2() : Node("string2nav2_node") {
  initPubSub();
  initClient();
}

void String2Nav2::initPubSub() {
  pub_speak_ = this->create_publisher<std_msgs::msg::String>("speak", 1);
  pub_initpose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 10);

  sub_voice2string_ = create_subscription<std_msgs::msg::String>(
      "voice2string", 1,
      std::bind(&String2Nav2::voice2stringCb, this, std::placeholders::_1));
}

void String2Nav2::initClient() {
  client_nav2_ = rclcpp_action::create_client<NavigateToPoseAction>(
      this, "navigate_to_pose");
}

void String2Nav2::voice2stringCb(
    const std_msgs::msg::String::ConstSharedPtr msg) {

  if (msg->data.find("シミュレータ") != std::string::npos ||
      msg->data.find("しみゅれーた") != std::string::npos) {
    // clang-format off
    if(std::system(
        "ros2 launch raspicat_gazebo raspicat_with_iscas_museum.launch.py gui:=false &"))
        exit(1);
    auto speak_str = std_msgs::msg::String();
    speak_str.data = "シミュレータ、立ち上げます！";
    pub_speak_->publish(speak_str);
    sleep(10);
    if(std::system(
        "ros2 service call /motor_power std_srvs/SetBool '{data: true}'"))
      exit(1);
    if(std::system("ros2 launch raspicat_navigation raspicat_nav2.launch.py &"))
      exit(1);
    // clang-format on
  }

  if (msg->data.find("初期位置") != std::string::npos ||
      msg->data.find("しょきいち") != std::string::npos) {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = -0.0738106;
    msg.pose.pose.position.y = 1.94351;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.00932562;
    msg.pose.pose.orientation.w = 0.999957;
    pub_initpose_->publish(msg);
    auto speak_str = std_msgs::msg::String();
    speak_str.data = "ロボットの自己位置推定の初期位置をセットしました";
    pub_speak_->publish(speak_str);
  }

  if (msg->data.find("自律移動") != std::string::npos ||
      msg->data.find("自律") != std::string::npos ||
      msg->data.find("移動") != std::string::npos) {

    send_goal(0, 8, 0);

    auto speak_str = std_msgs::msg::String();
    speak_str.data = "自律移動します";
    pub_speak_->publish(speak_str);
  }

  if (msg->data.find("終了") != std::string::npos ||
      msg->data.find("しゅうりょう") != std::string::npos) {
    if (std::system("killall -9 gzclient gzserver rviz2")) {
    }

    auto speak_str = std_msgs::msg::String();
    speak_str.data = "シミュレータのプログラム、おわりにします";
    pub_speak_->publish(speak_str);
  }
}

void String2Nav2::send_goal(const double x, const double y,
                            const double theta) {
  if (!client_nav2_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  auto goal_msg = NavigateToPoseAction::Goal();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();
  goal_msg.pose.header.frame_id = "map";

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
  auto goal_handle_future =
      client_nav2_->async_send_goal(goal_msg, send_goal_options);
}

} // namespace string2nav2
