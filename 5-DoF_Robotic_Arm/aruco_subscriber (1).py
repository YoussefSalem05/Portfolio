import rclpy
import math
import time
from rclpy.node import Node
from my_arm_interfaces.msg import ArucoDetection, TargetPose

class ArucoSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_subscriber')
        self.target_id = int(input("Enter target marker ID: "))
        self.subscription = self.create_subscription(ArucoDetection, '/aruco_detection', self.listener_callback, 10)
        self.target_pose_pub = self.create_publisher(TargetPose, '/target_pose', 10)
        
        # --- Memory & Thresholds ---
        self.last_x = None
        self.last_y = None
        # How far the block must move (in cm) before sending a new command
        self.MOVEMENT_THRESHOLD_CM = 0.5 
        
        # --- ESP32 Pick & Place Safety Cooldown ---
        self.last_command_time = 0.0
        # Wait 6 seconds for the ESP32 to finish its drop-off sequence
        self.COMMAND_COOLDOWN = 6.0 
        
        self.get_logger().info(f'Waiting for marker ID {self.target_id}')

    def listener_callback(self, msg: ArucoDetection):
        if msg.marker_id != self.target_id:
            return

        current_time = time.time()

        # 1. Check if the block has physically moved enough
        if self.last_x is not None and self.last_y is not None:
            distance_moved = math.sqrt((msg.x - self.last_x)**2 + (msg.y - self.last_y)**2)
            if distance_moved < self.MOVEMENT_THRESHOLD_CM:
                return # Block hasn't moved enough. Ignore.

        # 2. Check if the ESP32 is currently busy executing a sequence
        if current_time - self.last_command_time < self.COMMAND_COOLDOWN:
            return # ESP32 is busy. Ignore.

        # If we passed both checks, it's a valid new command! Update memory:
        self.last_x = msg.x
        self.last_y = msg.y
        self.last_command_time = current_time

        target = TargetPose()
        target.x = msg.x
        target.y = msg.y
        target.z = msg.z  
        target.roll = 0.0

        self.target_pose_pub.publish(target)
        self.get_logger().info(f'Published New Target to ESP32 -> X:{target.x:.1f}cm, Y:{target.y:.1f}cm, Z:{target.z:.1f}cm')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
