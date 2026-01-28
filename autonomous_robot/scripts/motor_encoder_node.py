#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
import threading
import time
import math

class MotorController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('motor_controller', anonymous=True)
        
        # Motor and encoder pins (from your configuration)
        self.LEFT_PWM = 13
        self.LEFT_DIR1 = 6
        self.LEFT_DIR2 = 5
        self.RIGHT_PWM = 12
        self.RIGHT_DIR1 = 23
        self.RIGHT_DIR2 = 24
        self.LEFT_ENC_A = 17
        self.LEFT_ENC_B = 27
        self.RIGHT_ENC_A = 16
        self.RIGHT_ENC_B = 20
        
        # Robot physical parameters (CRITICAL: These must match exactly in all nodes!)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.15)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0325)
        self.pulses_per_revolution = rospy.get_param('~pulses_per_revolution', 1320)  # CRITICAL: Set this correctly!
        
        # Motor parameters
        self.max_rpm = rospy.get_param('~max_rpm', 280)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 2.0)  # Reduced for better control
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 12.0)  # Reduced for better control
        self.pwm_frequency = rospy.get_param('~pwm_frequency', 10000)
        self.min_pwm_threshold = rospy.get_param('~min_pwm_threshold', 25)
        
        # PID parameters for better motor control
        self.use_pid = rospy.get_param('~use_pid', False)
        self.kp = rospy.get_param('~kp', 0.5)
        self.ki = rospy.get_param('~ki', 0.1)
        self.kd = rospy.get_param('~kd', 0.01)
        
        # Speed ramping parameters
        self.acceleration_limit = rospy.get_param('~acceleration_limit', 1.5)
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)
        
        # Encoder variables with thread safety
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.encoder_lock = threading.Lock()
        
        # Motor direction calibration (IMPORTANT: Test and set these!)
        self.left_motor_reverse = rospy.get_param('~left_motor_reverse', False)
        self.right_motor_reverse = rospy.get_param('~right_motor_reverse', False)
        
        # Encoder direction calibration (IMPORTANT: Test and set these!)
        self.left_encoder_reverse = rospy.get_param('~left_encoder_reverse', False)
        self.right_encoder_reverse = rospy.get_param('~right_encoder_reverse', False)
        
        # Velocity tracking for PID (if enabled)
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.current_left_vel = 0.0
        self.current_right_vel = 0.0
        self.left_integral = 0.0
        self.right_integral = 0.0
        self.left_prev_error = 0.0
        self.right_prev_error = 0.0
        
        # Setup GPIO
        self.setup_gpio()
        
        # ROS subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        
        # ROS publishers
        self.left_encoder_pub = rospy.Publisher('/left_encoder', Int32, queue_size=10)
        self.right_encoder_pub = rospy.Publisher('/right_encoder', Int32, queue_size=10)
        
        # ROS services
        self.reset_encoders_srv = rospy.Service('/reset_encoders', Empty, self.reset_encoders_callback)
        self.stop_motors_srv = rospy.Service('/emergency_stop', Empty, self.emergency_stop_callback)
        self.test_motors_srv = rospy.Service('/test_motors', Empty, self.test_motors_callback)
        
        # Control loop timer
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)  # 50Hz control loop
        
        rospy.loginfo("Motor Controller Node Started")
        rospy.loginfo(f"Wheel separation: {self.wheel_separation}m")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius}m")
        rospy.loginfo(f"Pulses per revolution: {self.pulses_per_revolution}")
        rospy.loginfo(f"Max speeds: linear={self.max_linear_speed}m/s, angular={self.max_angular_speed}rad/s")
        
    def setup_gpio(self):
        """Initialize GPIO pins with error handling"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor pins
            GPIO.setup(self.LEFT_PWM, GPIO.OUT)
            GPIO.setup(self.LEFT_DIR1, GPIO.OUT)
            GPIO.setup(self.LEFT_DIR2, GPIO.OUT)
            GPIO.setup(self.RIGHT_PWM, GPIO.OUT)
            GPIO.setup(self.RIGHT_DIR1, GPIO.OUT)
            GPIO.setup(self.RIGHT_DIR2, GPIO.OUT)
            
            # Setup encoder pins with pull-up resistors
            GPIO.setup(self.LEFT_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.LEFT_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.RIGHT_ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Create PWM objects
            self.left_pwm = GPIO.PWM(self.LEFT_PWM, self.pwm_frequency)
            self.right_pwm = GPIO.PWM(self.RIGHT_PWM, self.pwm_frequency)
            
            # Start PWM with 0% duty cycle
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            # Setup encoder interrupts with debouncing
            GPIO.add_event_detect(self.LEFT_ENC_A, GPIO.BOTH, 
                                callback=self.left_encoder_callback, bouncetime=1)
            GPIO.add_event_detect(self.RIGHT_ENC_A, GPIO.BOTH, 
                                callback=self.right_encoder_callback, bouncetime=1)
            
            rospy.loginfo("GPIO setup completed successfully")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup GPIO: {e}")
            raise
            
    def left_encoder_callback(self, channel):
        """Left encoder interrupt callback with improved direction detection"""
        try:
            with self.encoder_lock:
                a_state = GPIO.input(self.LEFT_ENC_A)
                b_state = GPIO.input(self.LEFT_ENC_B)
                
                # Quadrature decoding
                if a_state == b_state:
                    increment = 1
                else:
                    increment = -1
                
                # Apply encoder direction calibration
                if self.left_encoder_reverse:
                    increment = -increment
                    
                self.left_encoder_count += increment
                
        except Exception as e:
            rospy.logwarn(f"Left encoder callback error: {e}")
                
    def right_encoder_callback(self, channel):
        """Right encoder interrupt callback with improved direction detection"""
        try:
            with self.encoder_lock:
                a_state = GPIO.input(self.RIGHT_ENC_A)
                b_state = GPIO.input(self.RIGHT_ENC_B)
                
                # Quadrature decoding
                if a_state == b_state:
                    increment = 1
                else:
                    increment = -1
                
                # Apply encoder direction calibration
                if self.right_encoder_reverse:
                    increment = -increment
                    
                self.right_encoder_count += increment
                
        except Exception as e:
            rospy.logwarn(f"Right encoder callback error: {e}")
                
    def cmd_vel_callback(self, msg):
        """Convert twist message to wheel velocities with improved kinematics"""
        current_time = rospy.Time.now()
        
        # Apply velocity limits
        linear_vel = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
        
        # Apply acceleration ramping for smoother motion
        dt = (current_time - self.last_cmd_time).to_sec()
        if dt > 0 and dt < 0.1:  # Reasonable time delta
            # Limit linear acceleration
            max_linear_change = self.acceleration_limit * dt
            linear_change = linear_vel - self.last_linear_vel
            if abs(linear_change) > max_linear_change:
                linear_vel = self.last_linear_vel + math.copysign(max_linear_change, linear_change)
            
            # Limit angular acceleration
            max_angular_change = 3.0 * dt  # rad/s�
            angular_change = angular_vel - self.last_angular_vel
            if abs(angular_change) > max_angular_change:
                angular_vel = self.last_angular_vel + math.copysign(max_angular_change, angular_change)
        
        # Calculate wheel velocities using differential drive kinematics
        self.target_left_vel = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        self.target_right_vel = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Update last values
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel
        self.last_cmd_time = current_time
        
    def calculate_wheel_velocity(self, encoder_count, prev_count, dt):
        """Calculate wheel velocity from encoder data"""
        if dt <= 0:
            return 0.0
            
        # Calculate encoder difference
        encoder_diff = encoder_count - prev_count
        
        # Convert to distance
        distance = (encoder_diff / self.pulses_per_revolution) * (2 * math.pi * self.wheel_radius)
        
        # Calculate velocity
        velocity = distance / dt
        
        return velocity
        
    def pid_control(self, target_vel, current_vel, integral, prev_error, dt):
        """PID controller for motor speed"""
        if dt <= 0:
            return 0.0, integral, prev_error
            
        error = target_vel - current_vel
        integral += error * dt
        derivative = (error - prev_error) / dt
        
        # Anti-windup
        integral = max(-1.0, min(1.0, integral))
        
        output = self.kp * error + self.ki * integral + self.kd * derivative
        
        return output, integral, error
        
    def velocity_to_pwm(self, velocity, pid_output=0.0):
        """Convert wheel velocity to PWM value with optional PID correction"""
        if abs(velocity) < 0.01:  # Dead zone
            return 0.0
            
        # Base PWM calculation
        angular_velocity = velocity / self.wheel_radius
        rpm = (angular_velocity * 60) / (2 * math.pi)
        pwm_percent = min(abs(rpm) / self.max_rpm * 100, 100)
        
        # Add PID correction
        pwm_percent += pid_output
        
        # Apply minimum threshold
        if pwm_percent > 0 and pwm_percent < self.min_pwm_threshold:
            pwm_percent = self.min_pwm_threshold
            
        # Limit PWM range
        pwm_percent = max(-100, min(100, pwm_percent))
        
        return pwm_percent if velocity >= 0 else -pwm_percent
        
    def control_motor(self, motor, pwm_value):
        """Control individual motor with direction calibration"""
        # Apply motor direction calibration
        if motor == 'left' and self.left_motor_reverse:
            pwm_value = -pwm_value
        elif motor == 'right' and self.right_motor_reverse:
            pwm_value = -pwm_value
            
        if motor == 'left':
            if pwm_value > 0:
                GPIO.output(self.LEFT_DIR1, GPIO.HIGH)
                GPIO.output(self.LEFT_DIR2, GPIO.LOW)
                self.left_pwm.ChangeDutyCycle(abs(pwm_value))
            elif pwm_value < 0:
                GPIO.output(self.LEFT_DIR1, GPIO.LOW)
                GPIO.output(self.LEFT_DIR2, GPIO.HIGH)
                self.left_pwm.ChangeDutyCycle(abs(pwm_value))
            else:
                GPIO.output(self.LEFT_DIR1, GPIO.HIGH)
                GPIO.output(self.LEFT_DIR2, GPIO.HIGH)
                self.left_pwm.ChangeDutyCycle(0)
                
        elif motor == 'right':
            if pwm_value > 0:
                GPIO.output(self.RIGHT_DIR1, GPIO.HIGH)
                GPIO.output(self.RIGHT_DIR2, GPIO.LOW)
                self.right_pwm.ChangeDutyCycle(abs(pwm_value))
            elif pwm_value < 0:
                GPIO.output(self.RIGHT_DIR1, GPIO.LOW)
                GPIO.output(self.RIGHT_DIR2, GPIO.HIGH)
                self.right_pwm.ChangeDutyCycle(abs(pwm_value))
            else:
                GPIO.output(self.RIGHT_DIR1, GPIO.HIGH)
                GPIO.output(self.RIGHT_DIR2, GPIO.HIGH)
                self.right_pwm.ChangeDutyCycle(0)
                
    def control_loop(self, event):
        """Main control loop - runs at 50Hz"""
        current_time = rospy.Time.now()
        
        # Check for command timeout
        if (current_time - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self.target_left_vel = 0.0
            self.target_right_vel = 0.0
            
        # Get current encoder counts
        with self.encoder_lock:
            left_count = self.left_encoder_count
            right_count = self.right_encoder_count
            
        # Publish encoder data
        left_msg = Int32(data=left_count)
        right_msg = Int32(data=right_count)
        self.left_encoder_pub.publish(left_msg)
        self.right_encoder_pub.publish(right_msg)
        
        # Calculate PWM values
        if self.use_pid:
            # PID control (requires velocity feedback - more complex)
            dt = 0.02  # 50Hz
            
            # Calculate current velocities (simplified)
            # In practice, you'd need to track encoder differences over time
            
            left_pid, self.left_integral, self.left_prev_error = self.pid_control(
                self.target_left_vel, self.current_left_vel, 
                self.left_integral, self.left_prev_error, dt)
                
            right_pid, self.right_integral, self.right_prev_error = self.pid_control(
                self.target_right_vel, self.current_right_vel,
                self.right_integral, self.right_prev_error, dt)
                
            left_pwm = self.velocity_to_pwm(self.target_left_vel, left_pid)
            right_pwm = self.velocity_to_pwm(self.target_right_vel, right_pid)
        else:
            # Simple open-loop control
            left_pwm = self.velocity_to_pwm(self.target_left_vel)
            right_pwm = self.velocity_to_pwm(self.target_right_vel)
        
        # Control motors
        self.control_motor('left', left_pwm)
        self.control_motor('right', right_pwm)
        
    def reset_encoders_callback(self, req):
        """Reset encoder counts"""
        with self.encoder_lock:
            self.left_encoder_count = 0
            self.right_encoder_count = 0
        rospy.loginfo("Encoders reset")
        return EmptyResponse()
        
    def emergency_stop_callback(self, req):
        """Emergency stop all motors"""
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.control_motor('left', 0)
        self.control_motor('right', 0)
        rospy.logwarn("Emergency stop activated")
        return EmptyResponse()
        
    def test_motors_callback(self, req):
        """Test motor directions - IMPORTANT for calibration"""
        rospy.loginfo("Testing motors - robot should move forward for 2 seconds")
        rospy.loginfo("Left motor forward...")
        self.control_motor('left', 30)
        time.sleep(1)
        self.control_motor('left', 0)
        
        rospy.loginfo("Right motor forward...")
        self.control_motor('right', 30)
        time.sleep(1)
        self.control_motor('right', 0)
        
        rospy.loginfo("Both motors forward...")
        self.control_motor('left', 30)
        self.control_motor('right', 30)
        time.sleep(1)
        self.control_motor('left', 0)
        self.control_motor('right', 0)
        
        rospy.loginfo("Motor test complete - check directions and update parameters if needed")
        return EmptyResponse()
            
    def cleanup(self):
        """Clean up GPIO on shutdown"""
        try:
            self.control_motor('left', 0)
            self.control_motor('right', 0)
            time.sleep(0.1)
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()
            rospy.loginfo("Motor Controller cleaned up successfully")
        except Exception as e:
            rospy.logerr(f"Error during cleanup: {e}")

if __name__ == '__main__':
    controller = None
    try:
        controller = MotorController()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Motor Controller Node interrupted")
    except Exception as e:
        rospy.logerr(f"Motor Controller Error: {e}")
    finally:
        if controller:
            controller.cleanup()