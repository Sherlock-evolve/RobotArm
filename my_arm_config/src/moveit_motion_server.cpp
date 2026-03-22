#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_arm_config/action/plan_execute_pose.hpp"

namespace my_arm_config
{

using PlanExecutePose = my_arm_config::action::PlanExecutePose;
using GoalHandlePlanExecutePose = rclcpp_action::ServerGoalHandle<PlanExecutePose>;

class MoveItMotionServer : public rclcpp::Node
{
public:
  explicit MoveItMotionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("moveit_motion_server", options)
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "manipulator");
    // 你的系统约定 world_frame 与 base_link 重合，默认值也跟随这个约定
    base_frame_ = this->declare_parameter<std::string>("base_frame", "world_frame");
    // 统一以抓取点作为末端目标
    ee_link_ = this->declare_parameter<std::string>("ee_link", "grasp_point_link");
    // MoveIt默认容差设置
    // 对于5DOF机械臂，姿态容差需要设置较大，因为无法完全控制Roll
    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_ = this->declare_parameter<double>("orientation_tolerance", 0.5);
    // 使用近似IK：对5DOF机械臂更稳定，先找近似关节解再规划
    use_approx_ik_ = this->declare_parameter<bool>("use_approx_ik", true);
    
    action_server_ = rclcpp_action::create_server<PlanExecutePose>(
      this,
      "plan_execute_pose",
      std::bind(&MoveItMotionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveItMotionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveItMotionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveIt motion server ready for group '%s'", planning_group_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlanExecutePose::Goal> goal)
  {
    if (!goal) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    const std::string group = goal->planning_group.empty() ? planning_group_ : goal->planning_group;
    if (group == "arm") {
    const auto & pose = goal->target_pose.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z)) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: pose contains NaN/Inf");
        return rclcpp_action::GoalResponse::REJECT;
      }
    } else if (group == "gripper") {
      if (!std::isfinite(goal->gripper_position)) {
        RCLCPP_WARN(this->get_logger(), "Rejected goal: gripper_position contains NaN/Inf");
        return rclcpp_action::GoalResponse::REJECT;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: unknown planning_group '%s'", group.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    (void)goal_handle;
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (move_group_) {
      move_group_->stop();
    }
    if (gripper_move_group_) {
      gripper_move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    std::thread(&MoveItMotionServer::execute, this, goal_handle).detach();
  }

  void ensure_move_group()
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!move_group_) {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
      move_group_->setPoseReferenceFrame(base_frame_);
      move_group_->setEndEffectorLink(ee_link_);
    }
  }

  void ensure_gripper_group()
  {
    std::lock_guard<std::mutex> lock(move_group_mutex_);
    if (!gripper_move_group_) {
      gripper_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gripper");
    }
  }

  static std::string error_code_to_string(const moveit::core::MoveItErrorCode & code)
  {
    using moveit_msgs::msg::MoveItErrorCodes;
    switch (code.val) {
      case MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
      case MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
      case MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
      case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "INVALIDATED_BY_ENVIRONMENT_CHANGE";
      case MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
      case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_ACQUIRE_SENSOR_DATA";
      case MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
      case MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";
      case MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
      case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_CONSTRAINTS";
      case MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
      case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_CONSTRAINTS";
      case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";
      case MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
      case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
      case MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
      case MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";
      default:
        return "MoveIt error code " + std::to_string(code.val);
    }
  }

  void execute(const std::shared_ptr<GoalHandlePlanExecutePose> goal_handle)
  {
    auto result = std::make_shared<PlanExecutePose::Result>();
    auto goal = goal_handle->get_goal();

    const std::string group = goal->planning_group.empty() ? planning_group_ : goal->planning_group;
    const bool is_gripper = (group == "gripper");
    const bool is_arm = (group == "arm");

    if (!is_arm && !is_gripper) {
      result->success = false;
      result->message = "Unknown planning_group: " + group;
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (is_arm) {
    ensure_move_group();
    }
    if (is_gripper) {
      ensure_gripper_group();
    }

    geometry_msgs::msg::PoseStamped target_pose = goal->target_pose;
    if (target_pose.header.frame_id.empty()) {
      target_pose.header.frame_id = base_frame_;
    }
    target_pose.header.stamp = this->now();

    auto feedback = std::make_shared<PlanExecutePose::Feedback>();
    feedback->status = "Planning started";
    feedback->progress = 0.1f;
    goal_handle->publish_feedback(feedback);

    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      const double vel = std::clamp(goal->velocity_scaling, 0.01, 1.0);
      const double acc = std::clamp(goal->acceleration_scaling, 0.01, 1.0);

      if (is_arm) {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
        move_group_->setEndEffectorLink(ee_link_);
        move_group_->setGoalPositionTolerance(position_tolerance_);
        move_group_->setGoalOrientationTolerance(orientation_tolerance_);
      move_group_->setMaxVelocityScalingFactor(vel);
      move_group_->setMaxAccelerationScalingFactor(acc);
        move_group_->clearPoseTargets();

        // 使用近似IK：对5DOF机械臂更稳定
        // 先找近似关节解，再在关节空间规划，避免严格的姿态约束导致规划失败
        bool target_set = false;
        if (use_approx_ik_) {
          target_set = move_group_->setApproximateJointValueTarget(target_pose.pose, ee_link_);
          }
        if (!target_set) {
          // 如果近似IK失败，回退到标准方法
          move_group_->setPoseTarget(target_pose.pose, ee_link_);
        }

        RCLCPP_INFO(
          this->get_logger(),
          "Group=arm pose target (frame=%s): pos[%.3f, %.3f, %.3f], "
          "ori[%.3f, %.3f, %.3f, %.3f], "
          "tolerances: pos=%.3f m, ori=%.3f rad, use_approx_ik=%s",
          target_pose.header.frame_id.c_str(),
          target_pose.pose.position.x,
          target_pose.pose.position.y,
          target_pose.pose.position.z,
          target_pose.pose.orientation.x,
          target_pose.pose.orientation.y,
          target_pose.pose.orientation.z,
          target_pose.pose.orientation.w,
          position_tolerance_,
          orientation_tolerance_,
          use_approx_ik_ ? "true" : "false");
      } else {
        gripper_move_group_->setStartStateToCurrentState();
        gripper_move_group_->setMaxVelocityScalingFactor(vel);
        gripper_move_group_->setMaxAccelerationScalingFactor(acc);
        gripper_move_group_->setJointValueTarget("finger_joint", goal->gripper_position);
        RCLCPP_INFO(
          this->get_logger(),
          "Group=gripper joint target: finger_joint=%.3f",
          goal->gripper_position);
      }
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      planning_result = is_arm ? move_group_->plan(plan) : gripper_move_group_->plan(plan);
    }

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      result->message = "Planning failed: " + error_code_to_string(planning_result);
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled after planning";
      goal_handle->canceled(result);
      return;
    }

    feedback->status = "Planning succeeded";
    feedback->progress = 0.6f;
    goal_handle->publish_feedback(feedback);

    if (!goal->execute) {
      result->success = true;
      result->message = "Plan available (execution skipped)";
      goal_handle->succeed(result);
      return;
    }

    feedback->status = "Executing trajectory";
    feedback->progress = 0.8f;
    goal_handle->publish_feedback(feedback);

    moveit::core::MoveItErrorCode exec_result;
    {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      const bool wait = goal->wait_for_execution;
      if (wait) {
        exec_result = is_arm ? move_group_->execute(plan) : gripper_move_group_->execute(plan);
      } else {
        exec_result = is_arm ? move_group_->asyncExecute(plan) : gripper_move_group_->asyncExecute(plan);
      }
    }

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = false;
      result->message = "Execution failed: " + error_code_to_string(exec_result);
      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal canceled during execution";
      goal_handle->canceled(result);
      return;
    }

    feedback->status = "Execution completed";
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Motion completed successfully";
    goal_handle->succeed(result);
  }

  std::string planning_group_;
  std::string base_frame_;
  std::string ee_link_;
  double position_tolerance_{0.01};
  double orientation_tolerance_{0.5};
  bool use_approx_ik_{true};
  rclcpp_action::Server<PlanExecutePose>::SharedPtr action_server_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  std::mutex move_group_mutex_;
};

}  // namespace my_arm_config

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_arm_config::MoveItMotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
