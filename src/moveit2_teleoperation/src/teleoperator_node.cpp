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
  : Node("teleoperator_node")
  { 
    arm1_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "arm1");
    arm2_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "arm2");
    gripper1_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper1");
    gripper2_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}), "gripper2");

    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    robot1_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "/robot1/target_pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_pose_callback(*msg, *arm1_group_, arm1_mutex_, "arm1");
      }, options);

    robot2_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "/robot2/target_pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_pose_callback(*msg, *arm2_group_, arm2_mutex_, "arm2");
      }, options);

    gripper1_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot1/gripper_command", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gripper_callback(msg, *gripper1_group_, gripper1_mutex_, "gripper1");
      }, options);

    gripper2_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/robot2/gripper_command", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gripper_callback(msg, *gripper2_group_, gripper2_mutex_, "gripper2");
      }, options);
  }

private:

  // Function to control gripper.
  void gripper_callback(const std_msgs::msg::Bool::SharedPtr msg,
                        moveit::planning_interface::MoveGroupInterface& gripper_group,
                        std::mutex& gripper_mutex,
                        const std::string& gripper_name)
  {
    std::lock_guard<std::mutex> lock(gripper_mutex);
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
  void robot_pose_callback(const geometry_msgs::msg::Pose& target_pose,
                           moveit::planning_interface::MoveGroupInterface& arm_group, 
                           std::mutex& arm_mutex,
                           const std::string& arm_name)
  {
    std::lock_guard<std::mutex> lock(arm_mutex);
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
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm1_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm2_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper1_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper2_group_;
    
  std::mutex arm1_mutex_;
  std::mutex gripper1_mutex_;
  std::mutex arm2_mutex_;
  std::mutex gripper2_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
