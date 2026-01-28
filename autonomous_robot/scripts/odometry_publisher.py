#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
import math
import threading
import time

class OdometryPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('odometry_publisher', anonymous=True)
        
        # Robot physical parameters (should match motor controller)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.15)   # Distance between wheels in meters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0325)         # Wheel radius in meters
        self.pulses_per_revolution = rospy.get_param('~pulses_per_revolution', 1320)  # Encoder pulses per wheel revolution
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocity variables
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.velocity_window_size = 5
        self.velocity_history = []
        
        # Encoder variables
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0
        self.encoder_lock = threading.Lock()
        
        # Encoder data validation
        self.left_encoder_received = False
        self.right_encoder_received = False
        self.last_left_encoder_time = rospy.Time.now()
        self.last_right_encoder_time = rospy.Time.now()
        self.encoder_timeout = rospy.get_param('~encoder_timeout', 1.0)  # seconds
        
        # Time tracking
        self.last_time = rospy.Time.now()
        
        # Quality metrics
        self.max_wheel_speed = rospy.get_param('~max_wheel_speed', 2.0)  # m/s
        self.max_acceleration = rospy.get_param('~max_acceleration', 3.0)  # m/s²
        
        # ROS publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # ROS subscribers (from motor controller)
        self.left_enc_sub = rospy.Subscriber('/left_encoder', Int32, self.left_encoder_callback, queue_size=10)
        self.right_enc_sub = rospy.Subscriber('/right_encoder', Int32, self.right_encoder_callback, queue_size=10)
        
        # ROS services
        self.reset_odom_srv = rospy.Service('/reset_odometry', Empty, self.reset_odometry_service)
        
        # Parameters
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # Hz
        
        rospy.loginfo("Odometry Publisher Node Started")
        rospy.loginfo(f"Wheel separation: {self.wheel_separation}m")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius}m")
        rospy.loginfo(f"Pulses per revolution: {self.pulses_per_revolution}")
        
    def left_encoder_callback(self, msg):
        """Update left encoder count with validation"""
        with self.encoder_lock:
            # Validate encoder data
            encoder_diff = abs(msg.data - self.left_encoder_count)
            if encoder_diff > 1000 and self.left_encoder_received:  # Detect large jumps
                rospy.logwarn(f"Large left encoder jump detected: {encoder_diff}")
                return
                
            self.left_encoder_count = msg.data
            self.left_encoder_received = True
            self.last_left_encoder_time = rospy.Time.now()
            
    def right_encoder_callback(self, msg):
        """Update right encoder count with validation"""
        with self.encoder_lock:
            # Validate encoder data
            encoder_diff = abs(msg.data - self.right_encoder_count)
            if encoder_diff > 1000 and self.right_encoder_received:  # Detect large jumps
                rospy.logwarn(f"Large right encoder jump detected: {encoder_diff}")
                return
                
            self.right_encoder_count = msg.data
            self.right_encoder_received = True
            self.last_right_encoder_time = rospy.Time.now()
            
    def check_encoder_health(self):
        """Check if encoder data is being received"""
        current_time = rospy.Time.now()
        
        left_timeout = (current_time - self.last_left_encoder_time).to_sec() > self.encoder_timeout
        right_timeout = (current_time - self.last_right_encoder_time).to_sec() > self.encoder_timeout
        
        if left_timeout or right_timeout:
            if left_timeout:
                rospy.logwarn("Left encoder data timeout")
            if right_timeout:
                rospy.logwarn("Right encoder data timeout")
            return False
            
        return self.left_encoder_received and self.right_encoder_received
        
    def validate_motion(self, left_distance, right_distance, dt):
        """Validate motion data for reasonableness"""
        if dt <= 0 or dt > 1.0:  # Invalid time delta
            return False
            
        # Check for reasonable wheel speeds
        left_speed = abs(left_distance / dt)
        right_speed = abs(right_distance / dt)
        
        if left_speed > self.max_wheel_speed or right_speed > self.max_wheel_speed:
            rospy.logwarn(f"Excessive wheel speed detected: L={left_speed:.2f}, R={right_speed:.2f}")
            return False
            
        # Check for reasonable acceleration
        if hasattr(self, 'prev_left_speed') and hasattr(self, 'prev_right_speed'):
            left_accel = abs(left_speed - self.prev_left_speed) / dt
            right_accel = abs(right_speed - self.prev_right_speed) / dt
            
            if left_accel > self.max_acceleration or right_accel > self.max_acceleration:
                rospy.logwarn(f"Excessive acceleration detected: L={left_accel:.2f}, R={right_accel:.2f}")
                return False
                
        self.prev_left_speed = left_speed
        self.prev_right_speed = right_speed
        
        return True
        
    def calculate_odometry(self):
        """Calculate robot odometry from encoder data"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt <= 0:
            return
            
        # Check encoder health
        if not self.check_encoder_health():
            return
            
        with self.encoder_lock:
            # Calculate encoder differences
            left_diff = self.left_encoder_count - self.left_encoder_prev
            right_diff = self.right_encoder_count - self.right_encoder_prev
            
            # Update previous values
            self.left_encoder_prev = self.left_encoder_count
            self.right_encoder_prev = self.right_encoder_count
        
        # Convert encoder pulses to distance traveled
        left_distance = (left_diff / self.pulses_per_revolution) * (2 * math.pi * self.wheel_radius)
        right_distance = (right_diff / self.pulses_per_revolution) * (2 * math.pi * self.wheel_radius)
        
        # Validate motion data
        if not self.validate_motion(left_distance, right_distance, dt):
            return
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Calculate instantaneous velocities
        linear_velocity = distance / dt
        angular_velocity = delta_theta / dt
        
        # Apply moving average filter to velocities
        self.velocity_history.append((linear_velocity, angular_velocity))
        if len(self.velocity_history) > self.velocity_window_size:
            self.velocity_history.pop(0)
            
        # Calculate filtered velocities
        if len(self.velocity_history) > 0:
            linear_velocities = [v[0] for v in self.velocity_history]
            angular_velocities = [v[1] for v in self.velocity_history]
            self.linear_velocity = sum(linear_velocities) / len(linear_velocities)
            self.angular_velocity = sum(angular_velocities) / len(angular_velocities)
        
        # Update robot pose using more accurate integration
        if abs(delta_theta) < 1e-6:  # Straight line motion
            delta_x = distance * math.cos(self.theta + delta_theta / 2.0)
            delta_y = distance * math.sin(self.theta + delta_theta / 2.0)
        else:  # Curved motion - use exact integration
            radius = distance / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = self.normalize_angle(self.theta)
            
        # Publish odometry
        self.publish_odometry(current_time)
        
        self.last_time = current_time
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def calculate_covariance(self, linear_vel, angular_vel, dt):
        """Calculate dynamic covariance based on motion"""
        # Base covariance values
        base_linear_cov = 0.01
        base_angular_cov = 0.01
        
        # Increase covariance with speed and angular velocity
        speed_factor = 1.0 + abs(linear_vel) * 0.1
        angular_factor = 1.0 + abs(angular_vel) * 0.2
        
        # Increase covariance with time (integration uncertainty)
        time_factor = 1.0 + dt * 0.1
        
        linear_cov = base_linear_cov * speed_factor * time_factor
        angular_cov = base_angular_cov * angular_factor * time_factor
        
        return linear_cov, angular_cov
        
    def publish_odometry(self, current_time):
        """Publish odometry message and transform"""
        
        # Calculate dynamic covariance
        dt = (current_time - self.last_time).to_sec()
        linear_cov, angular_cov = self.calculate_covariance(self.linear_velocity, self.angular_velocity, dt)
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert from euler to quaternion)
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        # Dynamic covariance matrices
        odom_msg.pose.covariance = [
            linear_cov, 0, 0, 0, 0, 0,      # x
            0, linear_cov, 0, 0, 0, 0,      # y
            0, 0, 0, 0, 0, 0,               # z
            0, 0, 0, 0, 0, 0,               # roll
            0, 0, 0, 0, 0, 0,               # pitch
            0, 0, 0, 0, 0, angular_cov      # yaw
        ]
        
        velocity_cov = linear_cov * 0.1  # Velocity uncertainty is typically lower
        odom_msg.twist.covariance = [
            velocity_cov, 0, 0, 0, 0, 0,    # vx
            0, 0, 0, 0, 0, 0,               # vy
            0, 0, 0, 0, 0, 0,               # vz
            0, 0, 0, 0, 0, 0,               # vroll
            0, 0, 0, 0, 0, 0,               # vpitch
            0, 0, 0, 0, 0, angular_cov*0.1  # vyaw
        ]
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish transform if enabled
        if self.publish_tf:
            self.publish_transform(current_time, quaternion)
            
    def publish_transform(self, current_time, quaternion):
        """Publish TF transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
    def reset_odometry_service(self, req):
        """Service to reset odometry to origin"""
        with self.encoder_lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.left_encoder_prev = self.left_encoder_count
            self.right_encoder_prev = self.right_encoder_count
            
        # Clear velocity history
        self.velocity_history.clear()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        rospy.loginfo("Odometry reset to origin")
        return EmptyResponse()
        
    def get_odometry_status(self):
        """Get current odometry status for debugging"""
        with self.encoder_lock:
            status = {
                'position': (self.x, self.y, self.theta),
                'velocity': (self.linear_velocity, self.angular_velocity),
                'encoders': (self.left_encoder_count, self.right_encoder_count),
                'encoder_health': self.check_encoder_health()
            }
        return status
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"Odometry publisher running at {self.publish_rate} Hz")
        
        while not rospy.is_shutdown():
            self.calculate_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        odom_publisher.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Odometry Publisher Node Shutdown")
    except Exception as e:
        rospy.logerr(f"Odometry Publisher Error: {e}")