#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')
        
        self.target_sub = self.create_subscription(PoseStamped, '/landing_target/pose', self.target_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.last_target_time = self.get_clock().now()
        self.target_visible = False
        self.current_height = 0.0
        
        # PID Gains (Simple P)
        self.kp_xy = 0.5
        self.kp_z = 0.5
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.target_pose = None
        
        self.get_logger().info('Flight Controller Started')

    def target_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        self.target_visible = True
        self.target_pose = msg.pose

    def odom_callback(self, msg):
        self.current_height = msg.pose.pose.position.z

    def control_loop(self):
        # Check target timeout (1.0 sec)
        if (self.get_clock().now() - self.last_target_time).nanoseconds > 1e9:
            self.target_visible = False
            
        cmd = Twist()
        
        if self.target_visible and self.target_pose:
            # We see the target
            # Note: Target pose is in camera frame (X right, Y down, Z forward/depth)
            # We need to align camera center to target (x=0, y=0)
            
            # Error in image frame
            err_x = -self.target_pose.position.x # Camera x is Right, we want to move Left if target is Right? No.
            # If target is at +X (Right), we need to move Right. 
            # Wait, coordinate frames matter.
            # If we assume simple:
            # Camera X axis corresponds to Drone Y axis (Body frame)?
            # Camera Y axis corresponds to Drone X axis (Body frame)?
            # Standard Camera: Z forward, X right, Y down.
            # Drone Body: X forward, Y left, Z up.
            # If camera is looking down (Subject to mounting):
            # Let's assume Camera X aligns with Body Y, Camera Y aligns with Body -X.
            
            # Simple Logic: "Move towards target"
            
            # Body X cmd (Forward/Back) should reduce Camera Y error
            # If target is "Down" in image (+Y), it is "Behind" the drone? Or "In front"?
            # Let's assume standard down-facing camera: Top of image is "Front" of drone.
            # Image Y+ is Down (Bottom of image -> Rear of drone).
            # So if target.y > 0, target is behind. We need to move back (negative X speed).
            
            cmd.linear.x = -1.0 * self.target_pose.position.y * self.kp_xy
            
            # Body Y cmd (Left/Right) should reduce Camera X error
            # Image X+ is Right.
            # If target.x > 0, target is Right. We need to move Right (negative Y speed? usually Y is Left).
            # ROS convention: Y is Left.
            # So if target is Right, we move -Y.
            cmd.linear.y = -1.0 * self.target_pose.position.x * self.kp_xy
            
            # Vertical Control (Descend)
            # Desired depth is 0 (landed), but practically we want to descend until h < 0.2
            # Z cmd (Up/Down)
            if abs(self.target_pose.position.x) < 0.2 and abs(self.target_pose.position.y) < 0.2:
                 # Aligned, descend
                 cmd.linear.z = -0.5
            else:
                 # Maintain altitude or descend slowly
                 cmd.linear.z = -0.1
                 
        else:
            # Search Mode: Hover and rotate slowly
            cmd.linear.z = 0.0 # Maintain? Need PID for height hold really, but 0 vel might drift
            # For sim simple diff drive plugin, 0 vel stops it generally (unless gravity pulls it down fast without lift - simplified physics)
            cmd.angular.z = 0.2
            
            # Start off ground check
            if self.current_height < 2.0:
                 cmd.linear.z = 0.5 # Takeoff
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
