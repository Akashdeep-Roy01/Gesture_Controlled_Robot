#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    // Subscribe to target poses for arm1 and arm2
    robot1_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/robot1/target_pose", 10,
      std::bind(&TeleoperatorNode::robot1_pose_callback, this, std::placeholders::_1));

    robot2_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/robot2/target_pose", 10,
      std::bind(&TeleoperatorNode::robot2_pose_callback, this, std::placeholders::_1));

    gripper1_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot1/gripper_command", 10,
      std::bind(&TeleoperatorNode::gripper1_callback, this, std::placeholders::_1));

    gripper2_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
          "/robot2/gripper_command", 10,
          std::bind(&TeleoperatorNode::gripper2_callback, this, std::placeholders::_1));
  }

private:
  void robot1_pose_callback(const geometry_msgs::msg::Pose& target_pose)
  {
    move_arm(arm1_group_, target_pose, "arm1");
  }

  void robot2_pose_callback(const geometry_msgs::msg::Pose& target_pose)
  {
    move_arm(arm2_group_, target_pose, "arm2");
  }

  void gripper1_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::string target_pose = msg->data ? "close" : "open";
    RCLCPP_INFO(this->get_logger(), "Planning to %s gripper...", target_pose.c_str());
    // Set gripper target pose
    gripper1_group_.setNamedTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    gripper1_group_.plan(gripper_plan);
    gripper1_group_.execute(gripper_plan);
  }

  void gripper2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::string target_pose = msg->data ? "close" : "open";
    RCLCPP_INFO(this->get_logger(), "Planning to %s gripper...", target_pose.c_str());
    // Set gripper target pose
    gripper2_group_.setNamedTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    gripper2_group_.plan(gripper_plan);
    gripper2_group_.execute(gripper_plan);
  }

  void move_arm(moveit::planning_interface::MoveGroupInterface& arm_group, 
                const geometry_msgs::msg::Pose& target_pose, 
                const std::string& arm_name)
  {
    RCLCPP_INFO(this->get_logger(), "Received target pose for %s.", arm_name.c_str());
  
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

    // Compute Cartesian path to target pose
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; // 1 cm resolution for interpolation
    const double jump_threshold = 0.0; // Disable joint-space jumps
    double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) // Check if path is mostly complete
    {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;
      auto result = arm_group.execute(cartesian_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "%s: Cartesian path executed successfully.", arm_name.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "%s: Cartesian path execution failed with code: %d", 
                     arm_name.c_str(), result.val);
      }
    }
    else
    {
      arm_group.setPoseTarget(waypoints[0]);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "%s: Joint-space plan successful. Executing...", arm_name.c_str());
        auto result = arm_group.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "%s: Joint-space trajectory executed successfully.", arm_name.c_str());
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "%s: Joint-space execution failed with code: %d", 
                       arm_name.c_str(), result.val);
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "%s: Planning failed.", arm_name.c_str());
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot1_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot2_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper1_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper2_subscription_;
  moveit::planning_interface::MoveGroupInterface arm1_group_;
  moveit::planning_interface::MoveGroupInterface gripper1_group_;
  moveit::planning_interface::MoveGroupInterface arm2_group_;
  moveit::planning_interface::MoveGroupInterface gripper2_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

