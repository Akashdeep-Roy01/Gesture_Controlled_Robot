#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <mutex>

class TeleoperatorNode : public rclcpp::Node
{
public:
  TeleoperatorNode()
  : Node("teleoperator_node"),
    arm1_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "arm1"),
    arm2_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "arm2"),
    gripper1_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper1"),
    gripper2_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper2")
  {
    robot1_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "/robot1/target_pose", 10,
      std::bind(&TeleoperatorNode::robot1_pose_callback, this, std::placeholders::_1));

    robot2_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "/robot2/target_pose", 10,
      std::bind(&TeleoperatorNode::robot2_pose_callback, this, std::placeholders::_1));

    gripper1_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot1/gripper_command", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gripper_callback(msg, gripper1_group_, "gripper1");
      });

    gripper2_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot2/gripper_command", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gripper_callback(msg, gripper2_group_, "gripper2");
      });
  }

private:
  // Function to move robot 1 as per latest pose msg.
  void robot1_pose_callback(const geometry_msgs::msg::Pose& target_pose)
  {
    move_arm(arm1_group_, target_pose, "arm1");
  }

  // Function to move robot 2 as per latest pose msg.
  void robot2_pose_callback(const geometry_msgs::msg::Pose& target_pose)
  {
    move_arm(arm2_group_, target_pose, "arm2");
  }

  // Function to control gripper.
  void gripper_callback(const std_msgs::msg::Bool::SharedPtr msg,
                        moveit::planning_interface::MoveGroupInterface& gripper_group,
                        const std::string& gripper_name)
  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    std::string target_pose = msg->data ? "close" : "open"; // if thumb extended opens
    RCLCPP_INFO(get_logger(), "Planning to %s %s...", target_pose.c_str(), gripper_name.c_str());
    
    gripper_group.setNamedTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    if (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      auto result = gripper_group.execute(gripper_plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR_STREAM(get_logger(), gripper_name << ": Execution failed with code: " << result.val);
      }
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), gripper_name << ": Planning failed.");
    }
  }

  // Function to plan and move the arm. First cartesain planning is attempted, if fails then RRTConnect is used.
  void move_arm(moveit::planning_interface::MoveGroupInterface& arm_group,
                const geometry_msgs::msg::Pose& target_pose,
                const std::string& arm_name)
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    RCLCPP_INFO(get_logger(), "Received target pose for %s.", arm_name.c_str());
    arm_group.setStartStateToCurrentState();
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm_group.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory); // Cartesian Planner

    if (fraction > 0.9)
    {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;
      auto result = arm_group.execute(cartesian_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "%s: Cartesian path executed successfully.", arm_name.c_str());
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), arm_name << ": Cartesian path execution failed with code: " << result.val);
      }
    }
    else // use OMPL (RRTCnonect default)
    {
      arm_group.setPoseTarget(waypoints[0]);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success) {
        RCLCPP_INFO(get_logger(), "%s: Joint-space plan successful. Executing...", arm_name.c_str());
        auto result = arm_group.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(get_logger(), "%s: Joint-space trajectory executed successfully.", arm_name.c_str());
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), arm_name << ": Joint-space execution failed with code: " << result.val);
        }
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), arm_name << ": Planning failed.");
      }
    }
  }

  static constexpr double EEF_STEP = 0.01;
  static constexpr double JUMP_THRESHOLD = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot1_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot2_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper1_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper2_subscription_;
  moveit::planning_interface::MoveGroupInterface arm1_group_;
  moveit::planning_interface::MoveGroupInterface gripper1_group_;
  moveit::planning_interface::MoveGroupInterface arm2_group_;
  moveit::planning_interface::MoveGroupInterface gripper2_group_;
  std::mutex arm_mutex_;
  std::mutex gripper_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
