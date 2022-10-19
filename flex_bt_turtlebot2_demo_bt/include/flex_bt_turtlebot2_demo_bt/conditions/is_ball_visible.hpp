#ifndef FLEX_BT_TURTLEBOT_DEMO_BT__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define FLEX_BT_TURTLEBOT_DEMO_BT__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "ball_detector_msgs/msg/ball.hpp"
#include "ball_detector_msgs/msg/ball_list.hpp"
#include "geometry_msgs/msg/point.hpp"


namespace flex_bt_turtlebot
{

class IsBallVisibleCondition : public BT::ConditionNode
{
public:
  IsBallVisibleCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBallVisibleCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() {
    return {};
  }

private:
  void ballCallback(ball_detector_msgs::msg::BallList::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<ball_detector_msgs::msg::BallList>::SharedPtr ball_list_sub_;
  std::string ball_list_topic_;
  rclcpp::Time start_time_;

  bool is_ball_visible_;
  bool start_timer_;
};

}

#endif
