#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"

class RRGAutoRequest : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  RRGAutoRequest()
  : Node("rrg_auto_request")
  {
    action_client_ = rclcpp_action::create_client<ComputePathToPose>(
      this,
      "/compute_path_to_pose"
    );

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
      "/rrg_fixed_path",
      rclcpp::QoS(1).transient_local().reliable()
    );

    timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&RRGAutoRequest::sendGoalOnce, this)
    );

    RCLCPP_INFO(this->get_logger(), "RRG auto request node started.");
  }

private:
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_sent_ = false;

  void sendGoalOnce()
  {
    if (goal_sent_) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /compute_path_to_pose action server...");
      return;
    }

    goal_sent_ = true;

    ComputePathToPose::Goal goal_msg;

    goal_msg.planner_id = "RRG";
    goal_msg.use_start = true;

    goal_msg.start.header.frame_id = "map";
    goal_msg.start.pose.position.x = -1.9641;
    goal_msg.start.pose.position.y = 0.3022;
    goal_msg.start.pose.position.z = 0.0;
    goal_msg.start.pose.orientation.w = 1.0;

    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.pose.position.x = 1.7917;
    goal_msg.goal.pose.position.y = 0.3952;
    goal_msg.goal.pose.position.z = 0.0;
    goal_msg.goal.pose.orientation.w = 1.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending fixed RRG request: start=(%.3f, %.3f), goal=(%.3f, %.3f)",
      goal_msg.start.pose.position.x,
      goal_msg.start.pose.position.y,
      goal_msg.goal.pose.position.x,
      goal_msg.goal.pose.position.y
    );

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleComputePath::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "RRG goal was rejected.");
          return;
        }

        RCLCPP_INFO(this->get_logger(), "RRG goal was accepted.");
      };

    send_goal_options.feedback_callback =
      [this](
        GoalHandleComputePath::SharedPtr,
        const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
      {
        (void) feedback;
      };

    send_goal_options.result_callback =
      [this](const GoalHandleComputePath::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(
            this->get_logger(),
            "RRG path received. Number of poses: %zu. Planning time: %d.%09d s",
            result.result->path.poses.size(),
            result.result->planning_time.sec,
            result.result->planning_time.nanosec
          );

          path_publisher_->publish(result.result->path);
        } else {
          RCLCPP_ERROR(this->get_logger(), "RRG planning failed.");
        }
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RRGAutoRequest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
