#include <string>

#include "flex_bt_turtlebot2_demo_bt/conditions/is_ball_visible.hpp"

namespace flex_bt_turtlebot
{

IsBallVisibleCondition::IsBallVisibleCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  ball_list_topic_("/ball_detector/balls")
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  is_ball_visible_ = false;
  start_timer_ = true;
  start_time_ = steady_clock_.now();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  ball_list_sub_ = node_->create_subscription<ball_detector_msgs::msg::BallList>(
    ball_list_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBallVisibleCondition::ballCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsBallVisibleCondition::tick()
{
  callback_group_executor_.spin_some();
  if (start_timer_) {
    start_timer_ = false;
    start_time_ = steady_clock_.now();
  }

  if (is_ball_visible_ && steady_clock_.now().seconds() - start_time_.seconds() > 3.0) {
    is_ball_visible_ = false;
    start_timer_ = true;
    return BT::NodeStatus::SUCCESS;
  }
  else {
    return BT::NodeStatus::FAILURE;
  }
}

void IsBallVisibleCondition::ballCallback(ball_detector_msgs::msg::BallList::SharedPtr msg)
{
  is_ball_visible_ =  msg->balls.size() > 0;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<flex_bt_turtlebot::IsBallVisibleCondition>("IsBallVisible");
}
