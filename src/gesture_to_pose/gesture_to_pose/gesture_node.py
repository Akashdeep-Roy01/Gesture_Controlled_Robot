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

class GestureToPose(Node):
    def __init__(self):
        super().__init__('gesture_to_pose')
        self.image_publisher_ = self.create_publisher(Image, 'web_cam_image', 10)
        self.robot1_pose_publisher_ = self.create_publisher(Pose, '/robot1/target_pose', 10)
        self.robot2_pose_publisher_ = self.create_publisher(Pose, '/robot2/target_pose', 10)
        self.robot1_gripper_publisher_ = self.create_publisher(Bool, '/robot1/gripper_command', 10)
        self.robot2_gripper_publisher_ = self.create_publisher(Bool, '/robot2/gripper_command', 10)
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return
            
        # Smoothing buffers
        self.smooth_window = 5  # Number of frames for moving average
        self.left_hand_buffer = []
        self.right_hand_buffer = []
        
        self.roll_step = 0.1  # Radians per frame
        self.left_roll = 0.0
        self.right_roll = 0.0
        self.max_roll = np.pi*0.5
                
        self.timer = self.create_timer(0.1, self.timer_callback)  # ~10 FPS

    def apply_smoothing(self, is_left_hand, y_m, z_m):
        """Apply moving average smoothing to y and z coordinates."""
        buffer = self.left_hand_buffer if is_left_hand else self.right_hand_buffer
        
        buffer.append([y_m, z_m])
        if len(buffer) > self.smooth_window:
            buffer.pop(0)
        
        if buffer:
            avg_coords = np.mean(buffer, axis=0)
            return avg_coords[0], avg_coords[1]
        return y_m, z_m
    
    def is_finger_extended(self, hand_landmarks, idx_1, idx_2, idx_3):
        """Check if a finger is extended."""
        tip = hand_landmarks.landmark[idx_1]
        mcp = hand_landmarks.landmark[idx_2]
        base = hand_landmarks.landmark[idx_3]
        
        dist_tip_to_wrist = np.sqrt(
            (tip.x - base.x)**2 + (tip.y - base.y)**2
        )
        dist_mcp_to_wrist = np.sqrt(
            (mcp.x - base.x)**2 + (mcp.y - base.y)**2
        )
        return dist_tip_to_wrist > dist_mcp_to_wrist

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Webcam is not available")
            return
            
        ret, frame = self.cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
            
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        h, w, _ = frame.shape
        center_x = w // 2
        cv2.line(frame_bgr, (center_x, 0), (center_x, h), (255, 255, 255), 2)
        
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                index_tip = hand_landmarks.landmark[8]
                index_pixel_x, index_pixel_y = int(index_tip.x * w), int(index_tip.y * h)
                
                is_left_hand = handedness.classification[0].label == 'Left'
                in_correct_section = (is_left_hand and index_pixel_x <= center_x) or \
                                   (not is_left_hand and index_pixel_x > center_x)
                
                if not in_correct_section:
                    continue

                if is_left_hand:
                    # Map left half of image (0 to center_x) to -0.5 to 0.5
                    y_m = (index_tip.x * w / center_x) - 0.5
                else:
                    # Map right half of image (center_x to w) to -0.5 to 0.5
                    y_m = ((index_tip.x * w - center_x) / (w - center_x)) - 0.5
                
                z_m = (1 - index_tip.y) * 0.5 + 0.1 # z motion betwwen 0.1 to 0.6
                
                y_m_smooth, z_m_smooth = self.apply_smoothing(is_left_hand, y_m, z_m)
                
                gripper_closed = not self.is_finger_extended(hand_landmarks, 4, 2, 5)
                gripper_msg = Bool()
                gripper_msg.data = gripper_closed
                
                middle_extended = self.is_finger_extended(hand_landmarks, 12, 9, 0)
                pinky_extended = self.is_finger_extended(hand_landmarks, 20, 17, 0)
                
                current_roll = self.left_roll if is_left_hand else self.right_roll
                if middle_extended and not pinky_extended:
                    current_roll += self.roll_step
                elif not middle_extended and pinky_extended:
                    current_roll -= self.roll_step
                
                current_roll = np.clip(current_roll, -self.max_roll, self.max_roll)
                
                if is_left_hand:
                    self.left_roll = current_roll
                else:
                    self.right_roll = current_roll
                
               
                color = (0, 0, 255) if is_left_hand else (255, 0, 0)
                cv2.circle(frame_bgr, (index_pixel_x, index_pixel_y), 10, color, -1)
                self.mp_draw.draw_landmarks(frame_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                text_base_x = 20 if is_left_hand else center_x + 20
                text_base_y = 20
                line_height = 20

                cv2.putText(frame_bgr, f'{handedness.classification[0].label} Hand', 
                        (text_base_x, text_base_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame_bgr, f'Y: {y_m_smooth:.3f}m', 
                        (text_base_x, text_base_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame_bgr, f'Z: {z_m_smooth:.3f}m', 
                        (text_base_x, text_base_y + 2 * line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame_bgr, f'Gripper: {"Closed" if gripper_closed else "Open"}', 
                        (text_base_x, text_base_y + 3 * line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame_bgr, f'Roll: {current_roll:.3f} rad', 
                        (text_base_x, text_base_y + 4 * line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
         
                pose_msg = Pose()
                pose_msg.position.x = 0.3
                pose_msg.position.y = y_m_smooth + (-1.0 if is_left_hand else 1.0)
                pose_msg.position.z = z_m_smooth
                pose_msg.orientation.x = np.sin(current_roll*0.5)
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = np.cos(current_roll*0.5)
                
                if is_left_hand:
                    self.robot1_pose_publisher_.publish(pose_msg)
                    self.robot1_gripper_publisher_.publish(gripper_msg)
                else:
                    self.robot2_pose_publisher_.publish(pose_msg)
                    self.robot2_gripper_publisher_.publish(gripper_msg)
        
        try:
            self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(frame_bgr, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureToPose()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

