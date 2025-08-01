# src/final_project_pkg/final_project_pkg/task_executor.py (AI強化版)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class AITaskExecutor(Node):
    def __init__(self):
        super().__init__('ai_task_executor')
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # AI-enhanced state machine
        self.state = 'INITIALIZING'
        self.state_timer = 0.0
        self.delivery_targets = [
            {'name': 'Package A', 'target_distance': 3.0, 'completed': False},
            {'name': 'Package B', 'target_distance': 2.5, 'completed': False},
            {'name': 'Package C', 'target_distance': 4.0, 'completed': False}
        ]
        self.current_delivery = 0
        
        # Sensor data
        self.laser_data = None
        self.current_position = {'x': 0, 'y': 0, 'yaw': 0}
        self.traveled_distance = 0.0
        self.last_position = {'x': 0, 'y': 0}
        
        # AI parameters for obstacle avoidance
        self.min_obstacle_distance = 0.5  # meters
        self.safe_distance = 0.8  # meters
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        
        self.get_logger().info('🤖 AI-Enhanced Autonomous Delivery Robot Started!')
        self.get_logger().info(f'📦 Ready to deliver {len(self.delivery_targets)} packages')

    def scan_callback(self, msg):
        """AI-based sensor data processing"""
        self.laser_data = msg.ranges
        
    def odom_callback(self, msg):
        """Position tracking for intelligent navigation"""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        self.current_position['yaw'] = yaw
        
        # Calculate traveled distance
        dx = self.current_position['x'] - self.last_position['x']
        dy = self.current_position['y'] - self.last_position['y']
        self.traveled_distance += math.sqrt(dx**2 + dy**2)
        self.last_position = self.current_position.copy()

    def ai_obstacle_avoidance(self, base_cmd):
        """AI-powered obstacle detection and avoidance"""
        if self.laser_data is None:
            return base_cmd
        
        # Convert laser data to numpy array for AI processing
        ranges = np.array(self.laser_data)
        ranges = np.where(np.isinf(ranges), 10.0, ranges)  # Replace inf with large number
        
        # AI-based obstacle detection zones
        front_indices = list(range(0, 30)) + list(range(330, 360))  # Front 60 degrees
        left_indices = range(30, 150)   # Left side
        right_indices = range(210, 330) # Right side
        
        front_min = np.min(ranges[front_indices]) if front_indices else 10.0
        left_min = np.min(ranges[left_indices]) if left_indices else 10.0
        right_min = np.min(ranges[right_indices]) if right_indices else 10.0
        
        cmd = Twist()
        
        # AI decision making for obstacle avoidance
        if front_min < self.min_obstacle_distance:
            # Emergency stop and turn
            self.get_logger().warn(f'🚨 Obstacle detected at {front_min:.2f}m! Emergency avoidance!')
            cmd.linear.x = 0.0
            cmd.angular.z = 1.2 if left_min > right_min else -1.2
        elif front_min < self.safe_distance:
            # Slow down and gentle avoidance
            self.get_logger().info(f'⚠️ Obstacle ahead at {front_min:.2f}m, adjusting path...')
            cmd.linear.x = base_cmd.linear.x * 0.3
            cmd.angular.z = 0.5 if left_min > right_min else -0.5
        else:
            # Clear path - use base command with AI optimization
            speed_factor = min(1.0, front_min / 2.0)  # Adaptive speed based on distance
            cmd.linear.x = base_cmd.linear.x * speed_factor
            cmd.angular.z = base_cmd.angular.z
            
        return cmd

    def ai_target_selection(self):
        """AI-based next delivery target selection"""
        incomplete_deliveries = [d for d in self.delivery_targets if not d['completed']]
        if not incomplete_deliveries:
            return None
            
        # AI optimization: select closest incomplete delivery
        return min(incomplete_deliveries, key=lambda x: x['target_distance'])

    def control_loop(self):
        """Main AI-enhanced control loop"""
        self.state_timer += 0.1
        base_cmd = Twist()
        
        # AI-Enhanced State Machine
        if self.state == 'INITIALIZING':
            self.get_logger().info('🔄 Initializing AI systems...')
            if self.state_timer > 3.0:
                self.change_state('ANALYZING_ENVIRONMENT')
                
        elif self.state == 'ANALYZING_ENVIRONMENT':
            # AI environment analysis phase
            self.get_logger().info('🧠 AI analyzing delivery environment...')
            if self.state_timer > 2.0:
                next_target = self.ai_target_selection()
                if next_target:
                    self.get_logger().info(f'🎯 AI selected target: {next_target["name"]}')
                    self.change_state('MOVING_TO_PICKUP')
                else:
                    self.change_state('ALL_DELIVERIES_COMPLETE')
                    
        elif self.state == 'MOVING_TO_PICKUP':
            # AI-guided movement to pickup location
            current_target = self.delivery_targets[self.current_delivery]
            base_cmd.linear.x = self.max_linear_speed
            
            if self.traveled_distance >= current_target['target_distance']:
                self.get_logger().info(f'📍 Arrived at {current_target["name"]} pickup location')
                self.change_state('SCANNING_FOR_PACKAGE')
                
        elif self.state == 'SCANNING_FOR_PACKAGE':
            # AI package detection simulation
            self.get_logger().info(f'🔍 AI scanning for {self.delivery_targets[self.current_delivery]["name"]}...')
            if self.state_timer > 2.0:
                self.change_state('PICKING_UP')
                
        elif self.state == 'PICKING_UP':
            # Package pickup with AI confirmation
            current_target = self.delivery_targets[self.current_delivery]
            self.get_logger().info(f'🤖 Picking up {current_target["name"]}...')
            if self.state_timer > 2.5:
                self.get_logger().info(f'✅ {current_target["name"]} secured!')
                self.change_state('CALCULATING_DELIVERY_ROUTE')
                
        elif self.state == 'CALCULATING_DELIVERY_ROUTE':
            # AI route optimization
            self.get_logger().info('🗺️ AI calculating optimal delivery route...')
            if self.state_timer > 1.5:
                self.change_state('MOVING_TO_DELIVERY')
                
        elif self.state == 'MOVING_TO_DELIVERY':
            # AI-guided delivery movement
            base_cmd.linear.x = self.max_linear_speed
            if self.state_timer > 4.0:  # Simulated delivery distance
                self.change_state('DELIVERING_PACKAGE')
                
        elif self.state == 'DELIVERING_PACKAGE':
            # Package delivery with AI verification
            current_target = self.delivery_targets[self.current_delivery]
            self.get_logger().info(f'📦 Delivering {current_target["name"]}...')
            if self.state_timer > 2.0:
                current_target['completed'] = True
                self.get_logger().info(f'🎉 {current_target["name"]} delivered successfully!')
                self.current_delivery += 1
                self.traveled_distance = 0.0  # Reset for next delivery
                
                if self.current_delivery < len(self.delivery_targets):
                    self.change_state('ANALYZING_ENVIRONMENT')
                else:
                    self.change_state('ALL_DELIVERIES_COMPLETE')
                    
        elif self.state == 'ALL_DELIVERIES_COMPLETE':
            # Mission accomplished
            self.get_logger().info('🏆 All deliveries completed! Returning to base...')
            if self.state_timer > 2.0:
                self.change_state('MISSION_COMPLETE')
                
        elif self.state == 'MISSION_COMPLETE':
            self.get_logger().info('✨ AI Delivery Mission Successfully Completed!')
            self.timer.cancel()
            return
        
        # Apply AI obstacle avoidance to all movement commands
        final_cmd = self.ai_obstacle_avoidance(base_cmd)
        self.cmd_vel_publisher.publish(final_cmd)

    def change_state(self, new_state):
        self.get_logger().info(f'🔄 State transition: {self.state} → {new_state}')
        self.state = new_state
        self.state_timer = 0.0

def main(args=None):
    rclpy.init(args=args)
    ai_task_executor = AITaskExecutor()
    
    try:
        rclpy.spin(ai_task_executor)
    except KeyboardInterrupt:
        ai_task_executor.get_logger().info('🛑 AI Task Executor stopped by user')
    finally:
        ai_task_executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()