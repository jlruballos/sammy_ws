#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading

class RobotJointPublisher(Node):
    def __init__(self):
        super().__init__('robot_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Define all joints in the robot from XACRO
        self.joint_positions = {
            "RightHip": 0.0, "TorsoBow": 0.0, "TorsoTilt": 0.0, "RightChest": 0.0, 
            "HeadTurn": 0.0, "HeadNod": 0.0, "RightBicep": 0.0, "RightElbow": 0.0, 
            "RightKnee": 0.0, "RightAnkle": 0.0, "HeadTilt": 0.0, "LeftHip": 0.0, 
            "LeftKnee": 0.0, "LeftAnkle": 0.0, "LeftChest": 0.0, "LeftShoulder": 0.0, 
            "LeftBicep": 0.0, "LeftElbow": 0.0, "RightShoulder": 0.0
        }
        
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info("Robot joint publisher started")
        
    def publish_joint_states(self):
        # Create and publish a JointState message with all joints
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())
        self.publisher.publish(joint_state_msg)
    
    def move_joint(self, joint_name, target_position, duration=2.0):
        """Move a specific joint to the target position over the specified duration."""
        if joint_name not in self.joint_positions:
            self.get_logger().error(f"Joint {joint_name} not found in robot model")
            return
            
        start_position = self.joint_positions[joint_name]
        start_time = time.time()
        
        def update_position():
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                progress = min(elapsed / duration, 1.0)
                
                # Calculate intermediate position
                new_position = start_position + progress * (target_position - start_position)
                self.joint_positions[joint_name] = new_position
                
                # Log the current position periodically
                if int(elapsed * 10) % 5 == 0:  # Log every 0.5 seconds
                    self.get_logger().info(f"Moving {joint_name} to {new_position:.6f}")
                
                time.sleep(0.01)
            
            # Ensure we reach exactly the target position
            self.joint_positions[joint_name] = target_position
            self.get_logger().info(f"Final position of {joint_name} reached: {target_position}")
        
        # Start the joint movement in a separate thread
        threading.Thread(target=update_position, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    publisher = RobotJointPublisher()
    
    # Example: Move right elbow joint
    publisher.move_joint("RightElbow", 180.0, 10.0)
    publisher.move_joint("RightBicep", 90.0, 10.0)
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()