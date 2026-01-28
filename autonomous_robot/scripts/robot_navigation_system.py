#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import serial
import time
import threading
from math import sin, cos, pi, atan2, sqrt

class RobotNavigationSystem:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_navigation_system', anonymous=True)
        
        # Robot parameters
        self.wheel_radius = 0.033  # GA-25 370 motor wheel radius (meters)
        self.wheel_base = 0.2      # Distance between wheels (meters)
        self.max_speed = 1.0       # Maximum linear speed (m/s)
        self.max_angular_speed = 2.0  # Maximum angular speed (rad/s)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Sensor data variables
        self.imu_data = {
            'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
            'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
            'orientation': [0.0, 0.0, 0.0, 1.0]
        }
        self.lidar_data = {
            'ranges': [],
            'angle_min': 0.0,
            'angle_max': 0.0,
            'angle_increment': 0.0,
            'range_min': 0.0,
            'range_max': 0.0
        }
        
        # Navigation variables
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.path = []
        self.obstacles = []
        self.has_goal = False
        
        # Timing variables
        self.last_time = rospy.Time.now()
        
        # Serial connection for motor control (GA-25 370)
        self.motor_serial = None
        self.init_serial_connection()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.sensor_data_pub = rospy.Publisher('/sensor_data', String, queue_size=10)
        
        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Control loop rate
        self.rate = rospy.Rate(20)  # 20 Hz
        
        # Thread control
        self.running = True
        
        # Start threads
        self.control_thread = threading.Thread(target=self.control_loop)
        self.data_print_thread = threading.Thread(target=self.data_print_loop)
        self.control_thread.daemon = True
        self.data_print_thread.daemon = True
        self.control_thread.start()
        self.data_print_thread.start()
        
        rospy.loginfo("Robot Navigation System initialized")

    def init_serial_connection(self):
        """Initialize serial connection with error handling"""
        serial_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in serial_ports:
            try:
                self.motor_serial = serial.Serial(port, 9600, timeout=1)
                rospy.loginfo(f"Motor serial connection established on {port}")
                return
            except Exception as e:
                rospy.logdebug(f"Failed to connect to {port}: {e}")
                continue
        
        rospy.logwarn("Could not establish motor serial connection on any port")
        self.motor_serial = None

    def imu_callback(self, msg):
        """Process MPU 6050 IMU data"""
        try:
            self.imu_data['accel_x'] = msg.linear_acceleration.x
            self.imu_data['accel_y'] = msg.linear_acceleration.y
            self.imu_data['accel_z'] = msg.linear_acceleration.z
            
            self.imu_data['gyro_x'] = msg.angular_velocity.x
            self.imu_data['gyro_y'] = msg.angular_velocity.y
            self.imu_data['gyro_z'] = msg.angular_velocity.z
            
            # Update orientation using gyroscope data
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt > 0 and dt < 1.0:  # Sanity check on dt
                self.theta += self.imu_data['gyro_z'] * dt
                
                # Normalize angle
                while self.theta > pi:
                    self.theta -= 2 * pi
                while self.theta < -pi:
                    self.theta += 2 * pi
            
            self.last_time = current_time
            
        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")

    def lidar_callback(self, msg):
        """Process LIDAR data"""
        try:
            self.lidar_data['ranges'] = list(msg.ranges)
            self.lidar_data['angle_min'] = msg.angle_min
            self.lidar_data['angle_max'] = msg.angle_max
            self.lidar_data['angle_increment'] = msg.angle_increment
            self.lidar_data['range_min'] = msg.range_min
            self.lidar_data['range_max'] = msg.range_max
            
            # Process obstacles
            self.detect_obstacles()
        except Exception as e:
            rospy.logwarn(f"Error processing LIDAR data: {e}")

    def detect_obstacles(self):
        """Detect obstacles from LIDAR data"""
        self.obstacles = []
        ranges = self.lidar_data['ranges']
        
        if not ranges:
            return
            
        angle_min = self.lidar_data['angle_min']
        angle_increment = self.lidar_data['angle_increment']
        
        for i, range_val in enumerate(ranges):
            # Check if range is valid and within obstacle detection range
            if (not np.isnan(range_val) and not np.isinf(range_val) and 
                range_val < 2.0 and range_val > 0.1):
                
                angle = angle_min + i * angle_increment
                obs_x = self.x + range_val * cos(self.theta + angle)
                obs_y = self.y + range_val * sin(self.theta + angle)
                self.obstacles.append((obs_x, obs_y))

    def goal_callback(self, msg):
        """Set navigation goal"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        rospy.loginfo(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        
        # Simple path planning (straight line with obstacle avoidance)
        self.plan_path()

    def plan_path(self):
        """Simple path planning algorithm"""
        # For now, just use straight line to goal
        # In practice, you'd implement A* or RRT* here
        self.path = [(self.x, self.y), (self.goal_x, self.goal_y)]

    def obstacle_avoidance(self):
        """Simple obstacle avoidance using potential fields"""
        if not self.has_goal:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            return
            
        # Check if goal is reached
        goal_distance = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        if goal_distance < 0.2:  # Goal reached threshold
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.has_goal = False
            rospy.loginfo("Goal reached!")
            return
        
        # Attractive force towards goal
        if goal_distance > 0.1:
            goal_angle = atan2(self.goal_y - self.y, self.goal_x - self.x)
            attractive_force_x = 0.5 * cos(goal_angle)
            attractive_force_y = 0.5 * sin(goal_angle)
        else:
            attractive_force_x = 0.0
            attractive_force_y = 0.0
        
        # Repulsive force from obstacles
        repulsive_force_x = 0.0
        repulsive_force_y = 0.0
        
        for obs_x, obs_y in self.obstacles:
            obs_distance = sqrt((obs_x - self.x)**2 + (obs_y - self.y)**2)
            if obs_distance < 1.0 and obs_distance > 0.1:  # Within influence range
                repulsive_strength = 1.0 / (obs_distance**2)
                repulsive_angle = atan2(self.y - obs_y, self.x - obs_x)
                repulsive_force_x += repulsive_strength * cos(repulsive_angle)
                repulsive_force_y += repulsive_strength * sin(repulsive_angle)
        
        # Combine forces
        total_force_x = attractive_force_x + repulsive_force_x
        total_force_y = attractive_force_y + repulsive_force_y
        
        # Convert to velocity commands
        if abs(total_force_x) > 0.01 or abs(total_force_y) > 0.01:
            desired_angle = atan2(total_force_y, total_force_x)
            angle_diff = desired_angle - self.theta
            
            # Normalize angle difference
            while angle_diff > pi:
                angle_diff -= 2 * pi
            while angle_diff < -pi:
                angle_diff += 2 * pi
            
            # Calculate velocities
            force_magnitude = sqrt(total_force_x**2 + total_force_y**2)
            self.linear_vel = min(force_magnitude, self.max_speed)
            self.angular_vel = max(-self.max_angular_speed, 
                                 min(self.max_angular_speed, 2.0 * angle_diff))
        else:
            self.linear_vel = 0.0
            self.angular_vel = 0.0

    def control_motors(self):
        """Control GA-25 370 motors"""
        if self.motor_serial is None:
            return
        
        try:
            # Convert linear and angular velocities to wheel speeds
            left_wheel_speed = (self.linear_vel - self.angular_vel * self.wheel_base / 2) / self.wheel_radius
            right_wheel_speed = (self.linear_vel + self.angular_vel * self.wheel_base / 2) / self.wheel_radius
            
            # Convert to PWM values (assuming 0-255 range)
            left_pwm = int(max(-255, min(255, left_wheel_speed * 50)))  # Scale factor
            right_pwm = int(max(-255, min(255, right_wheel_speed * 50)))
            
            # Send motor commands
            command = f"M{left_pwm},{right_pwm}\n"
            self.motor_serial.write(command.encode())
            
        except Exception as e:
            rospy.logwarn(f"Failed to send motor command: {e}")

    def update_odometry(self):
        """Update robot odometry"""
        try:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            
            if dt > 0 and dt < 1.0:  # Sanity check
                # Update position
                self.x += self.linear_vel * cos(self.theta) * dt
                self.y += self.linear_vel * sin(self.theta) * dt
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            qx = 0.0
            qy = 0.0
            qz = sin(self.theta / 2)
            qw = cos(self.theta / 2)
            
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            
            odom_msg.twist.twist.linear.x = self.linear_vel
            odom_msg.twist.twist.angular.z = self.angular_vel
            
            self.odom_pub.publish(odom_msg)
            
            # Broadcast TF
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0), (qx, qy, qz, qw),
                current_time, "base_link", "odom"
            )
            
        except Exception as e:
            rospy.logwarn(f"Error updating odometry: {e}")

    def publish_cmd_vel(self):
        """Publish velocity commands"""
        try:
            cmd_msg = Twist()
            cmd_msg.linear.x = self.linear_vel
            cmd_msg.angular.z = self.angular_vel
            self.cmd_vel_pub.publish(cmd_msg)
        except Exception as e:
            rospy.logwarn(f"Error publishing cmd_vel: {e}")

    def control_loop(self):
        """Main control loop"""
        while not rospy.is_shutdown() and self.running:
            try:
                # Obstacle avoidance and navigation
                self.obstacle_avoidance()
                
                # Control motors
                self.control_motors()
                
                # Update odometry
                self.update_odometry()
                
                # Publish velocity commands
                self.publish_cmd_vel()
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")
                self.rate.sleep()

    def data_print_loop(self):
        """Print sensor data and robot state"""
        print_rate = rospy.Rate(2)  # Print at 2 Hz
        
        while not rospy.is_shutdown() and self.running:
            try:
                # Create sensor data string
                sensor_data = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'position': {'x': self.x, 'y': self.y, 'theta': self.theta},
                    'velocities': {'linear': self.linear_vel, 'angular': self.angular_vel},
                    'imu': self.imu_data,
                    'lidar': {
                        'num_points': len(self.lidar_data['ranges']),
                        'min_range': min(self.lidar_data['ranges']) if self.lidar_data['ranges'] else 0.0,
                        'obstacles_detected': len(self.obstacles)
                    },
                    'goal': {'x': self.goal_x, 'y': self.goal_y, 'has_goal': self.has_goal}
                }
                
                # Print to console
                print("\n" + "="*60)
                print(f"ROBOT NAVIGATION SYSTEM - {time.strftime('%H:%M:%S')}")
                print("="*60)
                print(f"Position: X={self.x:.3f}m, Y={self.y:.3f}m, �={np.degrees(self.theta):.1f}�")
                print(f"Velocities: Linear={self.linear_vel:.3f}m/s, Angular={self.angular_vel:.3f}rad/s")
                
                if self.has_goal:
                    goal_dist = sqrt((self.goal_x-self.x)**2 + (self.goal_y-self.y)**2)
                    print(f"Goal: X={self.goal_x:.3f}m, Y={self.goal_y:.3f}m")
                    print(f"Distance to Goal: {goal_dist:.3f}m")
                else:
                    print("Goal: No active goal")
                
                print(f"\nIMU (MPU 6050):")
                print(f"  Acceleration: X={self.imu_data['accel_x']:.3f}, Y={self.imu_data['accel_y']:.3f}, Z={self.imu_data['accel_z']:.3f} m/s�")
                print(f"  Gyroscope: X={self.imu_data['gyro_x']:.3f}, Y={self.imu_data['gyro_y']:.3f}, Z={self.imu_data['gyro_z']:.3f} rad/s")
                
                print(f"\nLIDAR:")
                if self.lidar_data['ranges']:
                    valid_ranges = [r for r in self.lidar_data['ranges'] if not np.isnan(r) and not np.isinf(r)]
                    if valid_ranges:
                        print(f"  Range Data Points: {len(valid_ranges)}")
                        print(f"  Min Range: {min(valid_ranges):.3f}m")
                        print(f"  Max Range: {max(valid_ranges):.3f}m")
                    print(f"  Obstacles Detected: {len(self.obstacles)}")
                else:
                    print("  No LIDAR data available")
                
                print(f"\nMotor Status (GA-25 370):")
                print(f"  Serial Connection: {'Connected' if self.motor_serial else 'Disconnected'}")
                print(f"  Motor Commands: Linear={self.linear_vel:.3f}m/s, Angular={self.angular_vel:.3f}rad/s")
                
                # Publish sensor data as ROS message
                sensor_msg = String()
                sensor_msg.data = str(sensor_data)
                self.sensor_data_pub.publish(sensor_msg)
                
                print_rate.sleep()
                
            except Exception as e:
                rospy.logwarn(f"Error in data print loop: {e}")
                print_rate.sleep()

    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down Robot Navigation System")
        
        # Stop threads
        self.running = False
        
        # Stop motors
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.control_motors()
        
        # Close serial connection
        if self.motor_serial:
            try:
                self.motor_serial.close()
            except:
                pass

def main():
    try:
        # Create robot navigation system
        robot = RobotNavigationSystem()
        
        # Register shutdown hook
        rospy.on_shutdown(robot.shutdown)
        
        # Keep the program running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Navigation System interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("Robot Navigation System stopped by user")
    except Exception as e:
        rospy.logerr(f"Error in Robot Navigation System: {str(e)}")

if __name__ == '__main__':
    main()