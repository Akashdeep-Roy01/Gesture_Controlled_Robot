#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import math

class FingerTracker(Node):
    def __init__(self):
        super().__init__('finger_tracker')
        self.image_publisher_ = self.create_publisher(Image, 'web_cam_image', 10)
        self.pose_publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        self.gripper_publisher_ = self.create_publisher(Bool, '/gripper_command', 10)
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return
            
        # Gripper control parameters
        self.last_command = None  # Track last command to avoid spamming
        
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        self.get_logger().info('Finger Tracker Node started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
            
        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        # Convert back to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Get landmarks: wrist (0), index finger tip (8), middle finger tip (12), middle finger MCP (9), thumb tip (4)
                wrist = hand_landmarks.landmark[0]
                index_tip = hand_landmarks.landmark[8]
                middle_tip = hand_landmarks.landmark[12]
                middle_mcp = hand_landmarks.landmark[9]
                thumb_tip = hand_landmarks.landmark[4]
                
                h, w, _ = frame.shape
                index_pixel_x, index_pixel_y = int(index_tip.x * w), int(index_tip.y * h)
                middle_pixel_x, middle_pixel_y = int(middle_tip.x * w), int(middle_tip.y * h)
                middle_mcp_pixel_y = int(middle_mcp.y * h)
                
                # Determine if middle finger is up (tip y-coordinate < MCP y-coordinate)
                is_middle_finger_up = middle_tip.y < middle_mcp.y
                command = Bool()
                command.data = is_middle_finger_up
                if self.last_command != command.data:
                    self.gripper_publisher_.publish(command)
                    self.get_logger().info(
                        f'Published gripper command: {"Open" if command.data else "Close"} '
                        f'(middle finger {"up" if is_middle_finger_up else "down"})')
                    self.last_command = command.data
                
                # Scale y-coordinate to [-0.3, 0.3] meters
                y_m = index_tip.x * 0.6 - 0.3  # scale: [0,1] -> [-0.3, 0.3]
                z_m = (1 - index_tip.y) * 0.5 + 0.1  # Invert and scale: [0,1] -> [0.6, 0.1]
                
                # Calculate angle between index finger and thumb for roll
                index_vec = np.array([index_tip.x - wrist.x, index_tip.y - wrist.y])
                thumb_vec = np.array([thumb_tip.x - wrist.x, thumb_tip.y - wrist.y])
                
                # Compute angle using arctangent
                angle_rad = math.atan2(np.cross(index_vec, thumb_vec), np.dot(index_vec, thumb_vec))
                roll_rad = np.clip(angle_rad, -math.pi/2, math.pi/2)  # Limit to ±90°
                
                # Convert roll to quaternion (rotation around x-axis)
                qw = math.cos(roll_rad / 2)
                qx = math.sin(roll_rad / 2)
                qy = 0.0
                qz = 0.0
                
                # Draw circles: red for index, blue for thumb, green for middle, yellow for middle MCP
                cv2.circle(frame_bgr, (index_pixel_x, index_pixel_y), 10, (0, 0, 255), -1)
                cv2.circle(frame_bgr, (int(thumb_tip.x * w), int(thumb_tip.y * h)), 10, (255, 0, 0), -1)
                cv2.circle(frame_bgr, (middle_pixel_x, middle_pixel_y), 10, (0, 255, 0), -1)
                cv2.circle(frame_bgr, (int(middle_mcp.x * w), int(middle_mcp.y * h)), 10, (0, 255, 255), -1)
                
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(frame_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Display y, z coordinates, roll angle, and middle finger state
                cv2.putText(frame_bgr, f'Y: {y_m:.3f}m', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame_bgr, f'Z: {z_m:.3f}m', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame_bgr, f'Roll: {math.degrees(roll_rad):.1f} deg', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame_bgr, f'Middle Finger: {"Up" if is_middle_finger_up else "Down"}', 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Create Pose message
                pose_msg = Pose()
                pose_msg.position.x = 0.3  # Fixed
                pose_msg.position.y = y_m
                pose_msg.position.z = z_m
                pose_msg.orientation.w = qw
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
                
                # Publish pose
                self.pose_publisher_.publish(pose_msg)
        
        # Publish image
        try:
            self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(frame_bgr, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FingerTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

