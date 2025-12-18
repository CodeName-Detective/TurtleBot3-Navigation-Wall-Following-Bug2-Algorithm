import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollow(Node):
    def __init__(self):
        super().__init__("Wall_Follow")
        
        
        # Initialize publishers and subscribers
        self.min_distance_to_wall = 0.5
        self.max_distance_to_wall = 0.7
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    
    def listener_callback(self, msg):
        self.twist_msg = Twist()
        
        
        front_wall_min_distance = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        right_wall_min_distance = min(msg.ranges[-40:-10])
            
        if front_wall_min_distance < 0.5:
            # When Obstacle is ahead of the object
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.4
        elif right_wall_min_distance > self.max_distance_to_wall:
            # Turn right
            self.twist_msg.linear.x = 0.1
            self.twist_msg.angular.z = -0.2
        elif right_wall_min_distance < self.min_distance_to_wall:
            # Turn left
            self.twist_msg.linear.x = 0.1
            self.twist_msg.angular.z = 0.2
        else:
            # Go forward
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0
                    
            
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        
        self.publisher_.publish(self.twist_msg)
        
        

def main():
    rclpy.init()
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()