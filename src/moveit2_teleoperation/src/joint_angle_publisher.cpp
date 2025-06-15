#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointAnglePublisher : public rclcpp::Node {
public:
  JointAnglePublisher() : Node("joint_angle_publisher") {
    robot1_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/robot1_joint_angles", 10);
    robot2_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/robot2_joint_angles", 10);
    subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JointAnglePublisher::joint_state_callback, this, std::placeholders::_1));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    sensor_msgs::msg::JointState robot1_msg, robot2_msg;
    robot1_msg.header = msg->header; // Copy timestamp
    robot2_msg.header = msg->header;

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i].find("robot1_") == 0) {
        robot1_msg.name.push_back(msg->name[i]);
        robot1_msg.position.push_back(msg->position[i]);
      } else if (msg->name[i].find("robot2_") == 0) {
        robot2_msg.name.push_back(msg->name[i]);
        robot2_msg.position.push_back(msg->position[i]);
      }
    }

    if (!robot1_msg.name.empty()) robot1_publisher_->publish(robot1_msg);
    if (!robot2_msg.name.empty()) robot2_publisher_->publish(robot2_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot1_publisher_, robot2_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointAnglePublisher>());
  rclcpp::shutdown();
  return 0;
}