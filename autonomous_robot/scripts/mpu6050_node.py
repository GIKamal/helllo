#!/usr/bin/env python3

import rospy
import smbus
import math
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse
import threading

class MPU6050Node:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mpu6050_node', anonymous=True)
        
        # MPU6050 I2C parameters
        self.bus = smbus.SMBus(1)  # I2C bus 1 (GPIO pins 2,3 on RPi)
        self.device_address = 0x68  # MPU6050 default I2C address
        
        # MPU6050 register addresses
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.INT_ENABLE = 0x38
        
        # Data registers
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.TEMP_OUT_H = 0x41
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        
        # Calibration variables
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.accel_offset_z = 0.0
        
        # Scale factors (adjust based on your MPU6050 configuration)
        self.gyro_scale = 131.0    # For �250�/s range
        self.accel_scale = 16384.0  # For �2g range
        
        # Complementary filter variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98  # Complementary filter coefficient
        self.last_time = time.time()
        
        # Error detection
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10
        self.sensor_healthy = True
        
        # Thread lock for data access
        self.data_lock = threading.Lock()
        
        # ROS publisher
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.imu_filtered_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        
        # ROS services
        self.calibrate_srv = rospy.Service('/imu/calibrate', Empty, self.calibrate_service)
        self.reset_orientation_srv = rospy.Service('/imu/reset_orientation', Empty, self.reset_orientation_service)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 100)  # Hz
        self.calibrate_on_start = rospy.get_param('~calibrate_on_start', True)
        
        # Initialize MPU6050
        self.initialize_mpu6050()
        
        # Calibrate if requested
        if self.calibrate_on_start:
            self.calibrate_sensors()
        
        rospy.loginfo("MPU6050 Node Started")
        
    def initialize_mpu6050(self):
        """Initialize MPU6050 sensor with error handling"""
        max_retries = 5
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                # Wake up the MPU6050
                self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 0)
                time.sleep(0.1)
                
                # Set sample rate divider (100Hz output rate)
                self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)
                
                # Set DLPF configuration (21Hz bandwidth)
                self.bus.write_byte_data(self.device_address, self.CONFIG, 0x04)
                
                # Set gyroscope configuration (�250�/s)
                self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 0)
                
                # Set accelerometer configuration (�2g)
                self.bus.write_byte_data(self.device_address, self.ACCEL_CONFIG, 0)
                
                # Enable data ready interrupt
                self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)
                
                # Test read to verify communication
                test_read = self.bus.read_byte_data(self.device_address, self.PWR_MGMT_1)
                
                rospy.loginfo("MPU6050 initialized successfully")
                self.sensor_healthy = True
                return
                
            except Exception as e:
                retry_count += 1
                rospy.logwarn(f"MPU6050 initialization attempt {retry_count} failed: {e}")
                time.sleep(0.5)
                
        rospy.logerr("Failed to initialize MPU6050 after multiple attempts")
        self.sensor_healthy = False
            
    def read_raw_data(self, addr):
        """Read raw 16-bit data from MPU6050 with error handling"""
        try:
            # Read high and low bytes
            high = self.bus.read_byte_data(self.device_address, addr)
            low = self.bus.read_byte_data(self.device_address, addr + 1)
            
            # Combine high and low bytes
            value = ((high << 8) | low)
            
            # Convert to signed 16-bit
            if value > 32768:
                value = value - 65536
                
            # Reset error counter on successful read
            self.consecutive_errors = 0
            self.sensor_healthy = True
            
            return value
            
        except Exception as e:
            self.consecutive_errors += 1
            if self.consecutive_errors > self.max_consecutive_errors:
                self.sensor_healthy = False
                rospy.logerr(f"MPU6050 sensor unhealthy after {self.consecutive_errors} consecutive errors")
            else:
                rospy.logwarn(f"MPU6050 read error: {e}")
            return 0
            
    def read_accelerometer(self):
        """Read accelerometer data"""
        if not self.sensor_healthy:
            return 0.0, 0.0, 0.0
            
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H) / self.accel_scale - self.accel_offset_x
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H) / self.accel_scale - self.accel_offset_y
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H) / self.accel_scale - self.accel_offset_z
        
        return acc_x, acc_y, acc_z
        
    def read_gyroscope(self):
        """Read gyroscope data"""
        if not self.sensor_healthy:
            return 0.0, 0.0, 0.0
            
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H) / self.gyro_scale - self.gyro_offset_x
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H) / self.gyro_scale - self.gyro_offset_y
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H) / self.gyro_scale - self.gyro_offset_z
        
        # Convert from degrees/s to rad/s
        gyro_x = math.radians(gyro_x)
        gyro_y = math.radians(gyro_y)
        gyro_z = math.radians(gyro_z)
        
        return gyro_x, gyro_y, gyro_z
        
    def read_temperature(self):
        """Read temperature data"""
        if not self.sensor_healthy:
            return 0.0
            
        temp_raw = self.read_raw_data(self.TEMP_OUT_H)
        temperature = (temp_raw / 340.0) + 36.53
        return temperature
        
    def calibrate_sensors(self):
        """Calibrate gyroscope and accelerometer offsets"""
        if not self.sensor_healthy:
            rospy.logwarn("Cannot calibrate: sensor unhealthy")
            return
            
        rospy.loginfo("Calibrating MPU6050... Keep the sensor stationary!")
        
        num_samples = 1000
        gyro_sum_x = gyro_sum_y = gyro_sum_z = 0
        accel_sum_x = accel_sum_y = accel_sum_z = 0
        
        valid_samples = 0
        
        for i in range(num_samples):
            # Read raw values
            gyro_x_raw = self.read_raw_data(self.GYRO_XOUT_H) / self.gyro_scale
            gyro_y_raw = self.read_raw_data(self.GYRO_YOUT_H) / self.gyro_scale
            gyro_z_raw = self.read_raw_data(self.GYRO_ZOUT_H) / self.gyro_scale
            
            accel_x_raw = self.read_raw_data(self.ACCEL_XOUT_H) / self.accel_scale
            accel_y_raw = self.read_raw_data(self.ACCEL_YOUT_H) / self.accel_scale
            accel_z_raw = self.read_raw_data(self.ACCEL_ZOUT_H) / self.accel_scale
            
            # Check for reasonable values (basic sanity check)
            if (abs(gyro_x_raw) < 100 and abs(gyro_y_raw) < 100 and abs(gyro_z_raw) < 100 and
                abs(accel_x_raw) < 4 and abs(accel_y_raw) < 4 and abs(accel_z_raw) < 4):
                
                gyro_sum_x += gyro_x_raw
                gyro_sum_y += gyro_y_raw
                gyro_sum_z += gyro_z_raw
                
                accel_sum_x += accel_x_raw
                accel_sum_y += accel_y_raw
                accel_sum_z += accel_z_raw
                
                valid_samples += 1
            
            time.sleep(0.001)
            
            if i % 100 == 0:
                rospy.loginfo(f"Calibration progress: {i}/{num_samples} (valid: {valid_samples})")
        
        if valid_samples < num_samples * 0.8:  # Require at least 80% valid samples
            rospy.logwarn(f"Calibration may be unreliable: only {valid_samples}/{num_samples} valid samples")
        
        # Calculate offsets
        self.gyro_offset_x = gyro_sum_x / valid_samples
        self.gyro_offset_y = gyro_sum_y / valid_samples
        self.gyro_offset_z = gyro_sum_z / valid_samples
        
        self.accel_offset_x = accel_sum_x / valid_samples
        self.accel_offset_y = accel_sum_y / valid_samples
        self.accel_offset_z = (accel_sum_z / valid_samples) - 1.0  # Subtract 1g for Z-axis
        
        rospy.loginfo("Calibration complete!")
        rospy.loginfo(f"Gyro offsets: X={self.gyro_offset_x:.4f}, Y={self.gyro_offset_y:.4f}, Z={self.gyro_offset_z:.4f}")
        rospy.loginfo(f"Accel offsets: X={self.accel_offset_x:.4f}, Y={self.accel_offset_y:.4f}, Z={self.accel_offset_z:.4f}")
        
    def complementary_filter(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt):
        """Apply complementary filter to fuse accelerometer and gyroscope data"""
        # Calculate angles from accelerometer (with proper axis alignment for robot)
        acc_roll = math.atan2(acc_y, math.sqrt(acc_x*acc_x + acc_z*acc_z))
        acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z))
        
        # Integrate gyroscope data
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt
        
        # Apply complementary filter for roll and pitch
        self.roll = self.alpha * self.roll + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * acc_pitch
        # Yaw relies only on gyroscope integration (no magnetometer)
        
        # Normalize angles
        self.roll = self.normalize_angle(self.roll)
        self.pitch = self.normalize_angle(self.pitch)
        self.yaw = self.normalize_angle(self.yaw)
        
        return self.roll, self.pitch, self.yaw
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        return [x, y, z, w]
        
    def calibrate_service(self, req):
        """Service to trigger calibration"""
        self.calibrate_sensors()
        return EmptyResponse()
        
    def reset_orientation_service(self, req):
        """Service to reset orientation angles"""
        with self.data_lock:
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
        rospy.loginfo("Orientation reset")
        return EmptyResponse()
        
    def publish_imu_data(self):
        """Read IMU data and publish ROS messages"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.1:  # Skip if dt is invalid or too large
            return
            
        try:
            # Read sensor data
            acc_x, acc_y, acc_z = self.read_accelerometer()
            gyro_x, gyro_y, gyro_z = self.read_gyroscope()
            
            # Create raw IMU message
            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = rospy.Time.now()
            imu_raw_msg.header.frame_id = self.frame_id
            
            # Linear acceleration (m/s�)
            imu_raw_msg.linear_acceleration.x = acc_x * 9.81
            imu_raw_msg.linear_acceleration.y = acc_y * 9.81
            imu_raw_msg.linear_acceleration.z = acc_z * 9.81
            
            # Angular velocity (rad/s)
            imu_raw_msg.angular_velocity.x = gyro_x
            imu_raw_msg.angular_velocity.y = gyro_y
            imu_raw_msg.angular_velocity.z = gyro_z
            
            # No orientation in raw data
            imu_raw_msg.orientation_covariance[0] = -1
            
            # Realistic covariance values
            imu_raw_msg.linear_acceleration_covariance = [
                0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 0.1
            ]
            
            imu_raw_msg.angular_velocity_covariance = [
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
            ]
            
            # Publish raw data
            self.imu_pub.publish(imu_raw_msg)
            
            # Apply complementary filter
            with self.data_lock:
                roll, pitch, yaw = self.complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt)
            
            # Create filtered IMU message
            imu_filtered_msg = Imu()
            imu_filtered_msg.header.stamp = rospy.Time.now()
            imu_filtered_msg.header.frame_id = self.frame_id
            
            # Copy acceleration and angular velocity
            imu_filtered_msg.linear_acceleration = imu_raw_msg.linear_acceleration
            imu_filtered_msg.angular_velocity = imu_raw_msg.angular_velocity
            
            # Add orientation from complementary filter
            quaternion = self.euler_to_quaternion(roll, pitch, yaw)
            imu_filtered_msg.orientation.x = quaternion[0]
            imu_filtered_msg.orientation.y = quaternion[1]
            imu_filtered_msg.orientation.z = quaternion[2]
            imu_filtered_msg.orientation.w = quaternion[3]
            
            # Copy covariances
            imu_filtered_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
            imu_filtered_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
            
            # Orientation covariance (higher for yaw due to drift)
            imu_filtered_msg.orientation_covariance = [
                0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 1.0  # Higher yaw uncertainty
            ]
            
            # Publish filtered data
            self.imu_filtered_pub.publish(imu_filtered_msg)
            
        except Exception as e:
            rospy.logerr(f"Error reading IMU data: {e}")
            
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            if self.sensor_healthy:
                self.publish_imu_data()
            else:
                # Try to reinitialize if sensor is unhealthy
                rospy.logwarn("Attempting to reinitialize unhealthy sensor...")
                self.initialize_mpu6050()
                time.sleep(1.0)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        mpu6050_node = MPU6050Node()
        mpu6050_node.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("MPU6050 Node Shutdown")
    except Exception as e:
        rospy.logerr(f"MPU6050 Node Error: {e}")