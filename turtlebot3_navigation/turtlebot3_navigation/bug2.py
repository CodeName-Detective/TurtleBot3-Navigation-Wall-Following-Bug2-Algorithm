import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf_transformations
import numpy as np
from math import atan2, sqrt, pi
import time

class Bug2(Node):
    def __init__(self):
        super().__init__("Bug2")
        
        self.theta_threshold_radians = 0.1
        self.distance_threshold_wall_follow_to_goal_seek = 0.1 # DIstance Threshold to line
        self.distance_threshold_goal_seek_to_wall_follow = 0.7 # Distance Threshold to Front Wall
        self.distance_threshold_to_goal = 0.2
        
        self.max_linear_velocity = 0.5 #meter/sec
        self.max_angular_velocity = 0.1 #rad/sec
        
        # Initialize publishers and subscribers
        self.min_distance_to_wall = 0.5
        self.max_distance_to_wall = 0.7
        
        self.declare_parameter('goal_position', [1.5, 1.5])
        self.goal_position = self.goal_position = self.get_parameter('goal_position').value
        
        self.no_initial_position = True
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_listener_callback,
            10)
        
        self.state = "goal_seek"
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.movement_listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def odom_listener_callback(self, odom):
        if self.no_initial_position:
            self.initial_position = [odom.pose.pose.position.x, odom.pose.pose.position.y]
            self.no_initial_position = False
        self.current_position = [odom.pose.pose.position.x, odom.pose.pose.position.y]
        theta_quaternion = odom.pose.pose.orientation
        _ , _, self.current_yaw = tf_transformations.euler_from_quaternion([theta_quaternion.x, theta_quaternion.y, theta_quaternion.z, theta_quaternion.w])
        
        self.current_linear_velocity = odom.twist.twist.linear.x
        self.current_angular_velocity = odom.twist.twist.angular.z
    
    ###############################
    def wrap_angle(self, angle):
        shift_by_pi = angle + pi
        wrap_0_to_2pi = shift_by_pi % (2*pi)
        wrap_minus_pi_to_pi = wrap_0_to_2pi - pi
        return wrap_minus_pi_to_pi
    
    def angular_velocity_PID_controller(self, delta_yaw, K_p=1.0, K_d=1.0, K_i=1.0):
        
        #angular_velocity = K_p * delta_yaw + K_d * (self._prev_error['angular_velocity']/self.time_period) + K_i * self._accumulated_error['angular_velocity']
        angular_velocity = K_p * delta_yaw
        # Clipping angular Velocity
        if angular_velocity > self.max_angular_velocity:
            return self.max_angular_velocity
        elif angular_velocity < -self.max_angular_velocity:
            return -self.max_angular_velocity
        else:
            return angular_velocity
    
    def linear_velocity_PID_controller(self, delta_distance, K_p=1.0, K_d=1.0, K_i=1.0):
        #linear_velocity = K_p * delta_distance + K_d * (self._prev_error['linear_velocity']/self.time_period) + K_i * self._accumulated_error['linear_velocity']
        linear_velocity = K_p * delta_distance
        # Clipping Linear Velocity
        if linear_velocity > self.max_linear_velocity:
            return self.max_linear_velocity
        elif linear_velocity < -self.max_linear_velocity:
            return -self.max_linear_velocity
        else:
            return linear_velocity
    ###############################
    
    
    def movement_listener_callback(self, msg):
        self.twist_msg = Twist()
        
        front_wall_min_distance = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
        right_wall_min_distance = min(msg.ranges[-40:-10])
        
        if not hasattr(self, 'current_angular_velocity'):
            return
        
        self.get_logger().info(f'Current Position: {self.current_position[0]}, {self.current_position[1]}')
        self.get_logger().info(f'Goal Position: {self.goal_position[0]}, {self.goal_position[1]}')
        
        if self.state == 'goal_seek':
            delta_x = self.goal_position[0] - self.current_position[0]
            delta_y = self.goal_position[1] - self.current_position[1]
            delta_theta_radians = self.wrap_angle(atan2(delta_y,delta_x)-self.current_yaw)
            distance = sqrt(delta_x**2 + delta_y**2)
            
            self.get_logger().info(self.state)
            
            if front_wall_min_distance < self.distance_threshold_goal_seek_to_wall_follow and distance > self.distance_threshold_to_goal:
                self.state = "wall_follow"
                self.twist_msg.linear.x = 0.09
                self.twist_msg.angular.z = 1.0

            elif abs(delta_theta_radians) > self.theta_threshold_radians:
                self.twist_msg.linear.x = 0.0
                # Proportional Controller
                self.twist_msg.angular.z = self.angular_velocity_PID_controller(delta_theta_radians, K_p=0.5, K_d=0.2, K_i=0.2)

            else:
                self.twist_msg.angular.z = 0.0
                if distance > self.distance_threshold_to_goal:
                    #linear_velocity = self.max_linear_velocity
                    self.twist_msg.linear.x = self.linear_velocity_PID_controller(distance, K_p=0.5, K_d=0.5, K_i=0.2)
                else:
                    self.twist_msg.linear.x = 0.0
                    self.get_logger().info('Goal Reached')
            
            self.get_logger().info(f'{self.twist_msg.linear.x}, {self.twist_msg.angular.z}')
            self.get_logger().info(self.state)
            self.get_logger().info('************')
        
        elif self.state == "wall_follow":
            
            x2, y2 = self.goal_position
            x1, y1 = self.initial_position
            x0, y0 = self.current_position
            
            numerator = np.abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1)
            denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            distance_to_line = np.array(numerator/denominator)
            
            self.get_logger().info(self.state)
            
            if distance_to_line < self.distance_threshold_wall_follow_to_goal_seek:
                self.state = "goal_seek"
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
            elif front_wall_min_distance < 0.5:
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
            self.get_logger().info(f'{self.twist_msg.linear.x}, {self.twist_msg.angular.z}')
            self.get_logger().info(self.state)
            self.get_logger().info('************')
        
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        
        self.publisher_.publish(self.twist_msg)
        
        

def main():
    rclpy.init()
    node = Bug2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()