#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class TeleoperatorNode : public rclcpp::Node
{
public:
  TeleoperatorNode()
  : Node("teleoperator_node"),
    arm_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "arm"),
    gripper_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper")
  {
    // Set planning parameters for arm
    arm_group_.setPlanningTime(5.0);
    arm_group_.setNumPlanningAttempts(10);
    arm_group_.setGoalPositionTolerance(0.01);
    arm_group_.setGoalOrientationTolerance(0.01);
    arm_group_.setStartStateToCurrentState();

    // Set planning parameters for gripper
    gripper_group_.setPlanningTime(5.0);
    gripper_group_.setNumPlanningAttempts(10);
    gripper_group_.setGoalJointTolerance(0.01);
    gripper_group_.setStartStateToCurrentState();

    // Subscribe to target pose for arm
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose", 10,
      std::bind(&TeleoperatorNode::pose_callback, this, std::placeholders::_1));

    // Subscribe to gripper command
    gripper_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/gripper_command", 10,
      std::bind(&TeleoperatorNode::gripper_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TeleoperatorNode initialized with arm and gripper control.");
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Received target pose.");

    geometry_msgs::msg::Pose current_pose = arm_group_.getCurrentPose().pose;

    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double dz = target_pose.position.z - current_pose.position.z;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.05)  // 5 cm threshold
    {
      RCLCPP_INFO(this->get_logger(), "Target is within 5 cm of current pose (%.3f m). No movement needed.", distance);
      return;  
    }
    
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

    // Compute Cartesian path to target pose
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; // 1 cm resolution for interpolation
    const double jump_threshold = 0.0; // Disable joint-space jumps
    double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) // Check if path is mostly complete
    {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;
      auto result = arm_group_.execute(cartesian_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Arm Cartesian path executed successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Arm Cartesian path execution failed with code: %d", result.val);
      }
    }
    else
    {
      arm_group_.setPoseTarget(waypoints[0]);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Arm joint-space plan successful. Executing...");
        auto result = arm_group_.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "Arm joint-space trajectory executed successfully.");
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Arm joint-space execution failed with code: %d", result.val);
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Arm planning failed.");
      }
    }
  }

  void gripper_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::string target_pose = msg->data ? "open" : "close";
    RCLCPP_INFO(this->get_logger(), "Planning to %s gripper...", target_pose.c_str());

    // Set gripper target pose
    gripper_group_.setNamedTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

    // Plan
    bool plan_success = (gripper_group_.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_success)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan for %s state", target_pose.c_str());
      return;
    }

    // Execute
    RCLCPP_INFO(this->get_logger(), "Gripper planning succeeded, executing...");
    auto execute_result = gripper_group_.execute(gripper_plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Gripper %s successfully!", target_pose.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute %s trajectory with code: %d",
                   target_pose.c_str(), execute_result.val);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_subscription_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
