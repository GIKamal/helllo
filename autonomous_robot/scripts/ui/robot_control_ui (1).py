#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QTabWidget, QGroupBox, QLabel, QPushButton, QComboBox, 
                            QSlider, QLCDNumber, QTextEdit, QScrollArea, QProgressBar)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor, QFont
import rospy # type: ignore
from std_msgs.msg import String, Bool, Float32, Int32 # type: ignore
from sensor_msgs.msg import LaserScan, Imu # type: ignore
from nav_msgs.msg import Odometry, OccupancyGrid, Path # type: ignore
from geometry_msgs.msg import Twist, PoseStamped, Vector3 # type: ignore
import math

# Try to import rviz, handle if not available
try:
    from rviz import bindings as rviz
    RVIZ_AVAILABLE = True
except ImportError:
    RVIZ_AVAILABLE = False
    print("Warning: rviz Python bindings not available. RViz tab will not be displayed.")

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS initialization
        rospy.init_node('robot_ui_node', anonymous=True)
        
        self.goal_history_combo = QComboBox(self)
        self.goal_history_combo.setObjectName("goalHistoryCombo")
        self.goal_history_combo.addItem("-- Select Previous Goal --")
        
        # Main window setup
        self.setWindowTitle('Autonomous Robot Control Panel')
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget and main layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        
        # Left panel (control and status)
        self.left_panel = QWidget()
        self.left_layout = QVBoxLayout(self.left_panel)
        self.left_layout.setContentsMargins(5, 5, 5, 5)
        
        # Right panel (visualization)
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(5, 5, 5, 5)
        
        # Add panels to main layout
        self.main_layout.addWidget(self.left_panel, 35)
        self.main_layout.addWidget(self.right_panel, 65)
        
        # Initialize UI components
        self.init_control_panel()
        self.init_sensor_status_panel()
        self.init_visualization_panel()
        self.init_custom_panel()
        self.init_ros_communication()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # 10 Hz update rate

        # Goal memory
        self.last_goals = []  # Store previous goals
        self.max_goal_history = 5  # How many to remember
        
        # Map click handling
        self.map_click_pos = None
        self.map_label.mousePressEvent = self.handle_map_click
        self.map_click_positions = []  # Store multiple click positions

        # Initialize sensor data storage
        self.mpu_data = {'accel': Vector3(), 'gyro': Vector3(), 'temp': 0.0}
        self.motor_data = {'left_rpm': 0, 'right_rpm': 0, 'left_encoder': 0, 'right_encoder': 0}
        self.system_data = {'voltage': 0.0, 'current': 0.0, 'temperature': 0.0}

    def init_custom_panel(self):
        custom_group = QGroupBox("Autonomous Robot Features")
        layout = QVBoxLayout()
        self.btn_custom = QPushButton("Activate Special Mode")
        self.btn_custom.clicked.connect(self.activate_special_mode)
        layout.addWidget(self.btn_custom)
        custom_group.setLayout(layout)
        self.left_layout.addWidget(custom_group)

    def activate_special_mode(self):
        rospy.loginfo("Special mode activated")

    def init_control_panel(self):
        """Initialize the control panel with tabs for different functionalities"""
        control_tabs = QTabWidget()
        
        # Tab 1: Manual Control
        manual_tab = QWidget()
        manual_layout = QVBoxLayout(manual_tab)
        
        # Mode selection
        mode_group = QGroupBox("Operation Mode")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Autonomous", "Manual", "Teleoperation", "Emergency Stop"])
        mode_layout.addWidget(self.mode_combo)
        mode_group.setLayout(mode_layout)
        
        # Velocity control
        vel_group = QGroupBox("Velocity Control")
        vel_layout = QVBoxLayout()
        
        # Linear velocity
        lin_vel_layout = QHBoxLayout()
        lin_vel_layout.addWidget(QLabel("Linear:"))
        self.lin_vel_slider = QSlider(Qt.Horizontal)
        self.lin_vel_slider.setRange(-100, 100)
        self.lin_vel_slider.setValue(0)
        lin_vel_layout.addWidget(self.lin_vel_slider)
        self.lin_vel_display = QLCDNumber()
        self.lin_vel_display.setDigitCount(5)
        lin_vel_layout.addWidget(self.lin_vel_display)
        vel_layout.addLayout(lin_vel_layout)
        
        # Angular velocity
        ang_vel_layout = QHBoxLayout()
        ang_vel_layout.addWidget(QLabel("Angular:"))
        self.ang_vel_slider = QSlider(Qt.Horizontal)
        self.ang_vel_slider.setRange(-100, 100)
        self.ang_vel_slider.setValue(0)
        ang_vel_layout.addWidget(self.ang_vel_slider)
        self.ang_vel_display = QLCDNumber()
        self.ang_vel_display.setDigitCount(5)
        ang_vel_layout.addWidget(self.ang_vel_display)
        vel_layout.addLayout(ang_vel_layout)
        
        vel_group.setLayout(vel_layout)
        
        # Navigation buttons
        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        
        button_row1 = QHBoxLayout()
        self.btn_forward = QPushButton("↑ Forward")
        self.btn_backward = QPushButton("↓ Backward")
        button_row1.addWidget(self.btn_forward)
        button_row1.addWidget(self.btn_backward)
        
        button_row2 = QHBoxLayout()
        self.btn_left = QPushButton("← Left")
        self.btn_stop = QPushButton("⏹ Stop")
        self.btn_right = QPushButton("→ Right")
        button_row2.addWidget(self.btn_left)
        button_row2.addWidget(self.btn_stop)
        button_row2.addWidget(self.btn_right)
        
        nav_layout.addLayout(button_row1)
        nav_layout.addLayout(button_row2)
        nav_group.setLayout(nav_layout)
        
        manual_layout.addWidget(mode_group)
        manual_layout.addWidget(vel_group)
        manual_layout.addWidget(nav_group)
        manual_layout.addStretch()
        
        # Tab 2: Autonomous Tasks
        auto_tab = QWidget()
        auto_layout = QVBoxLayout(auto_tab)

        goal_group = QGroupBox("Goal Position")
        goal_layout = QVBoxLayout()

        history_layout = QHBoxLayout()
        history_layout.addWidget(QLabel("Goal History:"))
        history_layout.addWidget(self.goal_history_combo)
        goal_layout.addLayout(history_layout)
        
        task_group = QGroupBox("Autonomous Tasks")
        task_layout = QVBoxLayout()
        
        self.btn_map = QPushButton("Create Map")
        self.btn_navigate = QPushButton("Navigate to Goal")
        self.btn_patrol = QPushButton("Start Patrol")
        self.btn_return = QPushButton("Return to Base")
        
        task_layout.addWidget(self.btn_map)
        task_layout.addWidget(self.btn_navigate)
        task_layout.addWidget(self.btn_patrol)
        task_layout.addWidget(self.btn_return)
        task_group.setLayout(task_layout)
        
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X:"))
        self.goal_x = QTextEdit()
        self.goal_x.setMaximumHeight(30)
        x_layout.addWidget(self.goal_x)
        
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y:"))
        self.goal_y = QTextEdit()
        self.goal_y.setMaximumHeight(30)
        y_layout.addWidget(self.goal_y)
        
        theta_layout = QHBoxLayout()
        theta_layout.addWidget(QLabel("θ:"))
        self.goal_theta = QTextEdit()
        self.goal_theta.setMaximumHeight(30)
        theta_layout.addWidget(self.goal_theta)
        
        goal_layout.addLayout(x_layout)
        goal_layout.addLayout(y_layout)
        goal_layout.addLayout(theta_layout)
        goal_group.setLayout(goal_layout)
        
        auto_layout.addWidget(task_group)
        auto_layout.addWidget(goal_group)
        auto_layout.addStretch()
        
        # Add tabs to control panel
        control_tabs.addTab(manual_tab, "Manual Control")
        control_tabs.addTab(auto_tab, "Autonomous")
        
        self.left_layout.addWidget(control_tabs)

    def save_goal_to_history(self, x, y, theta):
        """Store successful goals in memory"""
        try:
            x = float(x)
            y = float(y)
            theta = float(theta)
        except ValueError:
            return
        goal_str = f"X: {x:.2f}, Y: {y:.2f}, θ: {theta:.2f}"
        if goal_str in [self.goal_history_combo.itemText(i) for i in range(1, self.goal_history_combo.count())]:
            return
        
        self.last_goals.append((x, y, theta))
        self.goal_history_combo.addItem(goal_str)
        
        if len(self.last_goals) > self.max_goal_history:
            self.last_goals.pop(0)
            self.goal_history_combo.removeItem(1)

    def load_goal_from_history(self, index):
        """Load selected goal into fields"""
        if index == 0:
            return
            
        x, y, theta = self.last_goals[index-1]
        self.goal_x.setPlainText(f"{x:.2f}")
        self.goal_y.setPlainText(f"{y:.2f}")
        self.goal_theta.setPlainText(f"{theta:.2f}")

    def init_sensor_status_panel(self):
        """Initialize the sensor monitoring panel with MPU-6050, motor encoders, and system data"""
        # Create tabs for different sensor categories
        sensor_tabs = QTabWidget()
        
        # Tab 1: IMU Data (MPU-6050)
        imu_tab = QWidget()
        imu_layout = QVBoxLayout(imu_tab)
        
        # Accelerometer data
        accel_group = QGroupBox("Accelerometer (m/s²)")
        accel_layout = QVBoxLayout()
        
        accel_x_layout = QHBoxLayout()
        accel_x_layout.addWidget(QLabel("X:"))
        self.accel_x = QLCDNumber()
        self.accel_x.setDigitCount(6)
        self.accel_x.display("0.00")
        accel_x_layout.addWidget(self.accel_x)
        
        accel_y_layout = QHBoxLayout()
        accel_y_layout.addWidget(QLabel("Y:"))
        self.accel_y = QLCDNumber()
        self.accel_y.setDigitCount(6)
        self.accel_y.display("0.00")
        accel_y_layout.addWidget(self.accel_y)
        
        accel_z_layout = QHBoxLayout()
        accel_z_layout.addWidget(QLabel("Z:"))
        self.accel_z = QLCDNumber()
        self.accel_z.setDigitCount(6)
        self.accel_z.display("0.00")
        accel_z_layout.addWidget(self.accel_z)
        
        accel_layout.addLayout(accel_x_layout)
        accel_layout.addLayout(accel_y_layout)
        accel_layout.addLayout(accel_z_layout)
        accel_group.setLayout(accel_layout)
        
        # Gyroscope data
        gyro_group = QGroupBox("Gyroscope (rad/s)")
        gyro_layout = QVBoxLayout()
        
        gyro_x_layout = QHBoxLayout()
        gyro_x_layout.addWidget(QLabel("X:"))
        self.gyro_x = QLCDNumber()
        self.gyro_x.setDigitCount(6)
        self.gyro_x.display("0.00")
        gyro_x_layout.addWidget(self.gyro_x)
        
        gyro_y_layout = QHBoxLayout()
        gyro_y_layout.addWidget(QLabel("Y:"))
        self.gyro_y = QLCDNumber()
        self.gyro_y.setDigitCount(6)
        self.gyro_y.display("0.00")
        gyro_y_layout.addWidget(self.gyro_y)
        
        gyro_z_layout = QHBoxLayout()
        gyro_z_layout.addWidget(QLabel("Z:"))
        self.gyro_z = QLCDNumber()
        self.gyro_z.setDigitCount(6)
        self.gyro_z.display("0.00")
        gyro_z_layout.addWidget(self.gyro_z)
        
        gyro_layout.addLayout(gyro_x_layout)
        gyro_layout.addLayout(gyro_y_layout)
        gyro_layout.addLayout(gyro_z_layout)
        gyro_group.setLayout(gyro_layout)
        
        # Temperature from IMU
        temp_layout = QHBoxLayout()
        temp_layout.addWidget(QLabel("IMU Temperature (°C):"))
        self.imu_temp = QLCDNumber()
        self.imu_temp.setDigitCount(5)
        self.imu_temp.display("0.0")
        temp_layout.addWidget(self.imu_temp)
        
        imu_layout.addWidget(accel_group)
        imu_layout.addWidget(gyro_group)
        imu_layout.addLayout(temp_layout)
        imu_layout.addStretch()
        
        # Tab 2: Motor & Encoder Data
        motor_tab = QWidget()
        motor_layout = QVBoxLayout(motor_tab)
        
        # Motor RPM
        rpm_group = QGroupBox("Motor RPM")
        rpm_layout = QVBoxLayout()
        
        left_rpm_layout = QHBoxLayout()
        left_rpm_layout.addWidget(QLabel("Left Motor:"))
        self.left_rpm = QLCDNumber()
        self.left_rpm.setDigitCount(5)
        self.left_rpm.display("0")
        left_rpm_layout.addWidget(self.left_rpm)
        
        right_rpm_layout = QHBoxLayout()
        right_rpm_layout.addWidget(QLabel("Right Motor:"))
        self.right_rpm = QLCDNumber()
        self.right_rpm.setDigitCount(5)
        self.right_rpm.display("0")
        right_rpm_layout.addWidget(self.right_rpm)
        
        rpm_layout.addLayout(left_rpm_layout)
        rpm_layout.addLayout(right_rpm_layout)
        rpm_group.setLayout(rpm_layout)
        
        # Encoder counts
        encoder_group = QGroupBox("Encoder Counts")
        encoder_layout = QVBoxLayout()
        
        left_enc_layout = QHBoxLayout()
        left_enc_layout.addWidget(QLabel("Left Encoder:"))
        self.left_encoder = QLCDNumber()
        self.left_encoder.setDigitCount(8)
        self.left_encoder.display("0")
        left_enc_layout.addWidget(self.left_encoder)
        
        right_enc_layout = QHBoxLayout()
        right_enc_layout.addWidget(QLabel("Right Encoder:"))
        self.right_encoder = QLCDNumber()
        self.right_encoder.setDigitCount(8)
        self.right_encoder.display("0")
        right_enc_layout.addWidget(self.right_encoder)
        
        encoder_layout.addLayout(left_enc_layout)
        encoder_layout.addLayout(right_enc_layout)
        encoder_group.setLayout(encoder_layout)
        
        motor_layout.addWidget(rpm_group)
        motor_layout.addWidget(encoder_group)
        motor_layout.addStretch()
        
        # Tab 3: System Status & Power
        system_tab = QWidget()
        system_layout = QVBoxLayout(system_tab)
        
        # Real-time Power Consumption Data
        power_group = QGroupBox("Real-time Power Consumption")
        power_layout = QVBoxLayout()
        
        # RPLiDAR A1
        rplidar_group = QGroupBox("RPLiDAR A1")
        rplidar_layout = QHBoxLayout()
        
        rplidar_v_layout = QVBoxLayout()
        rplidar_v_layout.addWidget(QLabel("Voltage (V)"))
        self.rplidar_voltage = QLCDNumber()
        self.rplidar_voltage.setDigitCount(4)
        self.rplidar_voltage.display("0.0")
        rplidar_v_layout.addWidget(self.rplidar_voltage)
        
        rplidar_c_layout = QVBoxLayout()
        rplidar_c_layout.addWidget(QLabel("Current (A)"))
        self.rplidar_current = QLCDNumber()
        self.rplidar_current.setDigitCount(4)
        self.rplidar_current.display("0.0")
        rplidar_c_layout.addWidget(self.rplidar_current)
        
        rplidar_p_layout = QVBoxLayout()
        rplidar_p_layout.addWidget(QLabel("Power (W)"))
        self.rplidar_power = QLCDNumber()
        self.rplidar_power.setDigitCount(5)
        self.rplidar_power.display("0.0")
        rplidar_p_layout.addWidget(self.rplidar_power)
        
        rplidar_layout.addLayout(rplidar_v_layout)
        rplidar_layout.addLayout(rplidar_c_layout)
        rplidar_layout.addLayout(rplidar_p_layout)
        rplidar_group.setLayout(rplidar_layout)
        
        # Motors
        motors_group = QGroupBox("Motors")
        motors_layout = QHBoxLayout()
        
        motors_v_layout = QVBoxLayout()
        motors_v_layout.addWidget(QLabel("Voltage (V)"))
        self.motors_voltage = QLCDNumber()
        self.motors_voltage.setDigitCount(4)
        self.motors_voltage.display("0.0")
        motors_v_layout.addWidget(self.motors_voltage)
        
        motors_c_layout = QVBoxLayout()
        motors_c_layout.addWidget(QLabel("Current (A)"))
        self.motors_current = QLCDNumber()
        self.motors_current.setDigitCount(4)
        self.motors_current.display("0.0")
        motors_c_layout.addWidget(self.motors_current)
        
        motors_p_layout = QVBoxLayout()
        motors_p_layout.addWidget(QLabel("Power (W)"))
        self.motors_power = QLCDNumber()
        self.motors_power.setDigitCount(5)
        self.motors_power.display("0.0")
        motors_p_layout.addWidget(self.motors_power)
        
        motors_layout.addLayout(motors_v_layout)
        motors_layout.addLayout(motors_c_layout)
        motors_layout.addLayout(motors_p_layout)
        motors_group.setLayout(motors_layout)
        
        # Computing
        computing_group = QGroupBox("Computing System")
        computing_layout = QHBoxLayout()
        
        computing_v_layout = QVBoxLayout()
        computing_v_layout.addWidget(QLabel("Voltage (V)"))
        self.computing_voltage = QLCDNumber()
        self.computing_voltage.setDigitCount(4)
        self.computing_voltage.display("0.0")
        computing_v_layout.addWidget(self.computing_voltage)
        
        computing_c_layout = QVBoxLayout()
        computing_c_layout.addWidget(QLabel("Current (A)"))
        self.computing_current = QLCDNumber()
        self.computing_current.setDigitCount(4)
        self.computing_current.display("0.0")
        computing_c_layout.addWidget(self.computing_current)
        
        computing_p_layout = QVBoxLayout()
        computing_p_layout.addWidget(QLabel("Power (W)"))
        self.computing_power = QLCDNumber()
        self.computing_power.setDigitCount(5)
        self.computing_power.display("0.0")
        computing_p_layout.addWidget(self.computing_power)
        
        computing_layout.addLayout(computing_v_layout)
        computing_layout.addLayout(computing_c_layout)
        computing_layout.addLayout(computing_p_layout)
        computing_group.setLayout(computing_layout)
        
        # Total System Power
        total_layout = QHBoxLayout()
        total_layout.addWidget(QLabel("Total System Power (W):"))
        self.total_power = QLCDNumber()
        self.total_power.setDigitCount(6)
        self.total_power.display("0.0")
        total_layout.addWidget(self.total_power)
        
        power_layout.addWidget(rplidar_group)
        power_layout.addWidget(motors_group)
        power_layout.addWidget(computing_group)
        power_layout.addLayout(total_layout)
        power_group.setLayout(power_layout)
        
        # Robot position
        pos_group = QGroupBox("Robot Position")
        pos_layout = QVBoxLayout()
        
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X (m):"))
        self.pos_x = QLabel("0.00")
        self.pos_x.setFont(QFont("Arial", 12, QFont.Bold))
        x_layout.addWidget(self.pos_x)
        
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y (m):"))
        self.pos_y = QLabel("0.00")
        self.pos_y.setFont(QFont("Arial", 12, QFont.Bold))
        y_layout.addWidget(self.pos_y)
        
        theta_layout = QHBoxLayout()
        theta_layout.addWidget(QLabel("θ (°):"))
        self.pos_theta = QLabel("0.00")
        self.pos_theta.setFont(QFont("Arial", 12, QFont.Bold))
        theta_layout.addWidget(self.pos_theta)
        
        pos_layout.addLayout(x_layout)
        pos_layout.addLayout(y_layout)
        pos_layout.addLayout(theta_layout)
        pos_group.setLayout(pos_layout)
        
        # System temperature
        sys_temp_layout = QHBoxLayout()
        sys_temp_layout.addWidget(QLabel("System Temp (°C):"))
        self.sys_temp = QLCDNumber()
        self.sys_temp.setDigitCount(5)
        self.sys_temp.display("0.0")
        sys_temp_layout.addWidget(self.sys_temp)
        
        system_layout.addWidget(power_group)
        system_layout.addWidget(pos_group)
        system_layout.addLayout(sys_temp_layout)
        system_layout.addStretch()
        
        # Tab 4: Mapping Metrics
        mapping_tab = QWidget()
        mapping_layout = QVBoxLayout(mapping_tab)
        
        # Real-time Mapping Metrics
        mapping_group = QGroupBox("Real-time Mapping Metrics")
        mapping_metrics_layout = QVBoxLayout()
        
        # Map Resolution
        res_layout = QHBoxLayout()
        res_layout.addWidget(QLabel("Map Resolution (pixels/meter):"))
        self.map_resolution = QLCDNumber()
        self.map_resolution.setDigitCount(5)
        self.map_resolution.display("0.0")
        res_layout.addWidget(self.map_resolution)
        mapping_metrics_layout.addLayout(res_layout)
        
        # Coverage Percentage
        coverage_layout = QHBoxLayout()
        coverage_layout.addWidget(QLabel("Coverage Percentage of Known Area (%):"))
        self.coverage_percentage = QLCDNumber()
        self.coverage_percentage.setDigitCount(5)
        self.coverage_percentage.display("0.0")
        coverage_layout.addWidget(self.coverage_percentage)
        mapping_metrics_layout.addLayout(coverage_layout)
        
        # Wall Thickness Consistency
        wall_layout = QHBoxLayout()
        wall_layout.addWidget(QLabel("Wall Thickness Consistency (cm):"))
        self.wall_consistency = QLabel("±0.0")
        self.wall_consistency.setFont(QFont("Arial", 12, QFont.Bold))
        self.wall_consistency.setStyleSheet("color: blue; background-color: #f0f0f0; padding: 5px; border: 1px solid gray;")
        wall_layout.addWidget(self.wall_consistency)
        mapping_metrics_layout.addLayout(wall_layout)
        
        # Feature Detection Accuracy
        feature_layout = QHBoxLayout()
        feature_layout.addWidget(QLabel("Feature Detection Accuracy (%):"))
        self.feature_accuracy = QLCDNumber()
        self.feature_accuracy.setDigitCount(5)
        self.feature_accuracy.display("0.0")
        feature_layout.addWidget(self.feature_accuracy)
        mapping_metrics_layout.addLayout(feature_layout)
        
        # Current mapping status
        mapping_status_layout = QHBoxLayout()
        mapping_status_layout.addWidget(QLabel("Mapping Status:"))
        self.mapping_status = QLabel("Idle")
        self.mapping_status.setFont(QFont("Arial", 12, QFont.Bold))
        self.mapping_status.setStyleSheet("color: green; background-color: #f0f0f0; padding: 5px; border: 1px solid gray;")
        mapping_status_layout.addWidget(self.mapping_status)
        mapping_metrics_layout.addLayout(mapping_status_layout)
        
        # Mapping progress bar
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("Mapping Progress:"))
        self.mapping_progress = QProgressBar()
        self.mapping_progress.setRange(0, 100)
        self.mapping_progress.setValue(0)
        progress_layout.addWidget(self.mapping_progress)
        mapping_metrics_layout.addLayout(progress_layout)
        
        mapping_group.setLayout(mapping_metrics_layout)
        mapping_layout.addWidget(mapping_group)
        mapping_layout.addStretch()

        
        # Add tabs to sensor panel
        sensor_tabs.addTab(imu_tab, "IMU Data")
        sensor_tabs.addTab(motor_tab, "Motors")
        sensor_tabs.addTab(system_tab, "System")
        sensor_tabs.addTab(mapping_tab, "Mapping")
        
        self.left_layout.addWidget(sensor_tabs)

    def init_visualization_panel(self):
        """Initialize the visualization panel with Map and RViz (removed LIDAR tab)"""
        vis_tabs = QTabWidget()
        
        # Tab 1: Map View with Zoom Controls
        map_tab = QWidget()
        map_layout = QVBoxLayout(map_tab)
        
        # Zoom controls with current zoom display
        zoom_layout = QHBoxLayout()
        self.zoom_in_btn = QPushButton("Zoom In (+)")
        self.zoom_out_btn = QPushButton("Zoom Out (-)")
        self.zoom_reset_btn = QPushButton("Reset (R)")
        self.zoom_fit_btn = QPushButton("Fit to Window")
        
        # Initialize zoom variables
        self.zoom_factor = 1.0
        self.min_zoom = 0.1
        self.max_zoom = 10.0
        self.zoom_step = 0.2
        self.original_map_size = None
        
        # Zoom display label
        self.zoom_display = QLabel(f"Zoom: {self.zoom_factor:.1f}x")
        self.zoom_display.setMinimumWidth(80)
        
        # Connect zoom buttons
        self.zoom_in_btn.clicked.connect(self.zoom_in)
        self.zoom_out_btn.clicked.connect(self.zoom_out)
        self.zoom_reset_btn.clicked.connect(self.zoom_reset)
        self.zoom_fit_btn.clicked.connect(self.zoom_fit)
        
        # Add keyboard shortcuts info
        zoom_info = QLabel("Shortcuts: +/- keys, R=reset")
        zoom_info.setStyleSheet("color: gray; font-size: 10px;")
        
        zoom_layout.addWidget(self.zoom_in_btn)
        zoom_layout.addWidget(self.zoom_out_btn)
        zoom_layout.addWidget(self.zoom_reset_btn)
        zoom_layout.addWidget(self.zoom_fit_btn)
        zoom_layout.addWidget(self.zoom_display)
        zoom_layout.addStretch()
        zoom_layout.addWidget(zoom_info)
        
        map_layout.addLayout(zoom_layout)
        
        # Create scroll area for the map
        self.map_scroll_area = QScrollArea()
        self.map_scroll_area.setWidgetResizable(False)  # Important for zoom
        self.map_scroll_area.setAlignment(Qt.AlignCenter)
        self.map_scroll_area.setMinimumSize(800, 600)
        
        # Map label inside scroll area
        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setText("Click on map to set goal")
        self.map_label.setCursor(Qt.CrossCursor)
        self.map_label.setScaledContents(False)
        self.map_label.setStyleSheet("border: 1px solid gray;")
        
        # Set the map label as the scroll area's widget
        self.map_scroll_area.setWidget(self.map_label)
        map_layout.addWidget(self.map_scroll_area)
        
        # Tab 2: RViz Visualization
        if RVIZ_AVAILABLE:
            rviz_tab = QWidget()
            rviz_layout = QVBoxLayout(rviz_tab)
            
            self.rviz_frame = rviz.VisualizationFrame()
            self.rviz_frame.setMinimumSize(640, 480)
            self.rviz_frame.initialize()
            
            # Load default configuration or set up programmatically
            self.rviz_manager = self.rviz_frame.getManager()
            self.rviz_manager.setFixedFrame("map")
            
            # Add Map display
            map_display = self.rviz_manager.createDisplay("rviz/Map", "Map", True)
            map_display.subProp("Topic").setValue("/map")
            
            # Add LaserScan display
            laser_display = self.rviz_manager.createDisplay("rviz/LaserScan", "LaserScan", True)
            laser_display.subProp("Topic").setValue("/scan")
            laser_display.subProp("Size (m)").setValue(0.05)
            
            # Add Path display
            path_display = self.rviz_manager.createDisplay("rviz/Path", "GlobalPath", True)
            path_display.subProp("Topic").setValue("/move_base/DWAPlannerROS/global_plan")
            
            # Add Pose display
            pose_display = self.rviz_manager.createDisplay("rviz/Pose", "RobotPose", True)
            pose_display.subProp("Topic").setValue("/odom")
            
            rviz_layout.addWidget(self.rviz_frame)
            vis_tabs.addTab(rviz_tab, "RViz")
        
        # Add tabs to visualization panel
        vis_tabs.addTab(map_tab, "Map")
        
        self.right_layout.addWidget(vis_tabs)

        # Costmaps panel
        self.costmap_tabs = QTabWidget()
        self.global_costmap_label = QLabel()
        self.local_costmap_label = QLabel()
        self.global_costmap_label.setMinimumSize(400, 400)
        self.local_costmap_label.setMinimumSize(400, 400)
        self.global_costmap_label.setText("Global Costmap")
        self.local_costmap_label.setText("Local Costmap")

        costmap_tab = QWidget()
        costmap_layout = QHBoxLayout(costmap_tab)
        costmap_layout.addWidget(self.global_costmap_label)
        costmap_layout.addWidget(self.local_costmap_label)
        self.costmap_tabs.addTab(costmap_tab, "Costmaps")

        self.right_layout.addWidget(self.costmap_tabs)
        
        # Enable keyboard shortcuts for zoom
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts for zoom"""
        if event.key() == Qt.Key_Plus or event.key() == Qt.Key_Equal:
            self.zoom_in()
        elif event.key() == Qt.Key_Minus:
            self.zoom_out()
        elif event.key() == Qt.Key_R:
            self.zoom_reset()
        else:
            super().keyPressEvent(event)

    def zoom_in(self):
        """Zoom in on the map"""
        if self.zoom_factor < self.max_zoom:
            self.zoom_factor = min(self.max_zoom, self.zoom_factor + self.zoom_step)
            self.update_map_zoom()
            self.system_messages.append(f"Zoomed in: {self.zoom_factor:.1f}x")

    def zoom_out(self):
        """Zoom out on the map"""
        if self.zoom_factor > self.min_zoom:
            self.zoom_factor = max(self.min_zoom, self.zoom_factor - self.zoom_step)
            self.update_map_zoom()
            self.system_messages.append(f"Zoomed out: {self.zoom_factor:.1f}x")

    def zoom_reset(self):
        """Reset zoom to 1:1"""
        self.zoom_factor = 1.0
        self.update_map_zoom()
        self.system_messages.append("Zoom reset to 1:1")

    def zoom_fit(self):
        """Fit map to scroll area size"""
        if hasattr(self, 'original_map_pixmap') and self.original_map_pixmap:
            scroll_size = self.map_scroll_area.size()
            # Account for scrollbars
            available_width = scroll_size.width() - 20
            available_height = scroll_size.height() - 20
            
            # Calculate zoom factor to fit
            width_ratio = available_width / self.original_map_pixmap.width()
            height_ratio = available_height / self.original_map_pixmap.height()
            
            # Use the smaller ratio to ensure the entire map fits
            fit_zoom = min(width_ratio, height_ratio)
            fit_zoom = max(self.min_zoom, min(self.max_zoom, fit_zoom))
            
            self.zoom_factor = fit_zoom
            self.update_map_zoom()
            self.system_messages.append(f"Fit to window: {self.zoom_factor:.1f}x")

    def update_map_zoom(self):
        """Update the map display with current zoom factor"""
        if hasattr(self, 'original_map_pixmap') and self.original_map_pixmap:
            # Calculate new size
            new_width = int(self.original_map_pixmap.width() * self.zoom_factor)
            new_height = int(self.original_map_pixmap.height() * self.zoom_factor)
            
            # Scale the pixmap
            scaled_pixmap = self.original_map_pixmap.scaled(
                new_width, new_height, 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            
            # Update the label
            self.map_label.setPixmap(scaled_pixmap)
            self.map_label.resize(scaled_pixmap.size())
            
            # Update zoom display
            self.zoom_display.setText(f"Zoom: {self.zoom_factor:.1f}x")
            
            # Update button states
            self.zoom_in_btn.setEnabled(self.zoom_factor < self.max_zoom)
            self.zoom_out_btn.setEnabled(self.zoom_factor > self.min_zoom)

    def handle_map_click(self, event):
        """Convert mouse click to map coordinates with zoom consideration"""
        if not hasattr(self, 'last_map_data'):
            self.system_messages.append("Error: No map data received yet")
            return
        
        try:
            # Get click position relative to the scaled image
            click_pos = event.pos()
            pixmap = self.map_label.pixmap()
            if not pixmap:
                return
            
            # Convert click coordinates to original image coordinates
            x_px = click_pos.x() / self.zoom_factor
            y_px = click_pos.y() / self.zoom_factor
            
            # Convert to map coordinates
            origin = self.last_map_data.info.origin.position
            resolution = self.last_map_data.info.resolution
            
            map_x = origin.x + (x_px * resolution)
            map_y = origin.y + ((self.last_map_data.info.height - y_px) * resolution)
            
            # Update goal fields
            self.goal_x.setPlainText(f"{map_x:.2f}")
            self.goal_y.setPlainText(f"{map_y:.2f}")
            self.goal_theta.setPlainText("0.0")
            
            # Store click position (in original coordinates)
            self.map_click_positions.append((x_px, y_px))
            if len(self.map_click_positions) > 5:
                self.map_click_positions.pop(0)
            
            # Redraw the map with click markers
            self.redraw_map_with_markers()
            
            self.system_messages.append(f"Goal set at ({map_x:.2f}, {map_y:.2f})")
            
        except Exception as e:
            rospy.logerr(f"Map click error: {e}")

    def redraw_map_with_markers(self):
        """Redraw the map with current markers and zoom"""
        if hasattr(self, 'original_map_image'):
            # Start with the original image
            image = self.original_map_image.copy()
            
            # Draw robot position if available
            if hasattr(self, 'current_pose') and hasattr(self, 'last_map_data'):
                self.draw_robot_on_image(image)
            
            # Draw click markers
            self.draw_click_markers_on_image(image)
            
            # Convert to pixmap and store as original
            self.original_map_pixmap = QPixmap.fromImage(image)
            
            # Apply zoom
            self.update_map_zoom()

    def draw_robot_on_image(self, image):
        """Draw robot position and orientation on image"""
        try:
            painter = QPainter(image)
            painter.setPen(QColor(0, 255, 0))  # Green for robot
            painter.setBrush(QColor(0, 255, 0, 180))
            
            # Convert robot position to pixel coordinates
            robot_x_px = int((self.current_pose.position.x - self.last_map_data.info.origin.position.x) / self.last_map_data.info.resolution)
            robot_y_px = int((self.current_pose.position.y - self.last_map_data.info.origin.position.y) / self.last_map_data.info.resolution)
            robot_y_px = self.last_map_data.info.height - robot_y_px  # Flip Y coordinate
            
            # Draw robot as a circle
            painter.drawEllipse(robot_x_px - 5, robot_y_px - 5, 10, 10)
            
            # Draw robot orientation
            q = self.current_pose.orientation
            theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            end_x = robot_x_px + int(15 * math.cos(theta))
            end_y = robot_y_px - int(15 * math.sin(theta))  # Negative because Y is flipped
            painter.drawLine(robot_x_px, robot_y_px, end_x, end_y)
            
            painter.end()
        except Exception as e:
            rospy.logerr(f"Error drawing robot: {e}")

    def draw_click_markers_on_image(self, image):
        """Draw click markers on image"""
        try:
            painter = QPainter(image)
            painter.setPen(QColor(255, 0, 0))
            painter.setBrush(QColor(255, 0, 0, 128))
            
            for x_px, y_px in self.map_click_positions:
                painter.drawEllipse(int(x_px)-5, int(y_px)-5, 10, 10)
            
            painter.end()
        except Exception as e:
            rospy.logerr(f"Error drawing click markers: {e}")

    def init_ros_communication(self):
        """Initialize ROS publishers and subscribers"""
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mode_pub = rospy.Publisher('/operation_mode', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Map and navigation subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_callback)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.global_plan_callback)
        
        # MPU-6050 IMU data subscriber
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Motor and encoder data subscribers
        rospy.Subscriber('/motor/left_rpm', Int32, self.left_rpm_callback)
        rospy.Subscriber('/motor/right_rpm', Int32, self.right_rpm_callback)
        rospy.Subscriber('/encoder/left_count', Int32, self.left_encoder_callback)
        rospy.Subscriber('/encoder/right_count', Int32, self.right_encoder_callback)
        
        # System monitoring subscribers
        rospy.Subscriber('/system/voltage', Float32, self.voltage_callback)
        rospy.Subscriber('/system/current', Float32, self.current_callback)
        rospy.Subscriber('/system/temperature', Float32, self.temperature_callback)
        rospy.Subscriber('/system_messages', String, self.sysmsg_callback)
        
        # Real-time power consumption data subscribers
        rospy.Subscriber('/power/rplidar_voltage', Float32, self.rplidar_voltage_callback)
        rospy.Subscriber('/power/rplidar_current', Float32, self.rplidar_current_callback)
        rospy.Subscriber('/power/rplidar_power', Float32, self.rplidar_power_callback)
        rospy.Subscriber('/power/motors_voltage', Float32, self.motors_voltage_callback)
        rospy.Subscriber('/power/motors_current', Float32, self.motors_current_callback)
        rospy.Subscriber('/power/motors_power', Float32, self.motors_power_callback)
        rospy.Subscriber('/power/computing_voltage', Float32, self.computing_voltage_callback)
        rospy.Subscriber('/power/computing_current', Float32, self.computing_current_callback)
        rospy.Subscriber('/power/computing_power', Float32, self.computing_power_callback)
        rospy.Subscriber('/power/total_power', Float32, self.total_power_callback)
        
        # Real-time mapping metrics subscribers
        rospy.Subscriber('/mapping/resolution', Float32, self.map_resolution_callback)
        rospy.Subscriber('/mapping/coverage_percentage', Float32, self.coverage_percentage_callback)
        rospy.Subscriber('/mapping/wall_consistency', String, self.wall_consistency_callback)
        rospy.Subscriber('/mapping/feature_accuracy', Float32, self.feature_accuracy_callback)
        rospy.Subscriber('/mapping/status', String, self.mapping_status_callback)
        rospy.Subscriber('/mapping/progress', Float32, self.mapping_progress_callback)
        
        # Connect UI elements
        self.mode_combo.currentTextChanged.connect(self.mode_changed)
        self.lin_vel_slider.valueChanged.connect(self.velocity_changed)
        self.ang_vel_slider.valueChanged.connect(self.velocity_changed)
        
        self.btn_forward.pressed.connect(lambda: self.move_robot(0.5, 0))
        self.btn_backward.pressed.connect(lambda: self.move_robot(-0.5, 0))
        self.btn_left.pressed.connect(lambda: self.move_robot(0, 0.5))
        self.btn_right.pressed.connect(lambda: self.move_robot(0, -0.5))
        self.btn_stop.pressed.connect(lambda: self.move_robot(0, 0))
        
        self.btn_navigate.clicked.connect(self.send_goal)
        self.goal_history_combo.currentIndexChanged.connect(self.load_goal_from_history)

    # IMU and sensor callbacks remain the same
    def imu_callback(self, msg):
        """Process IMU data from MPU-6050"""
        try:
            self.mpu_data['accel'] = msg.linear_acceleration
            self.accel_x.display(f"{msg.linear_acceleration.x:.2f}")
            self.accel_y.display(f"{msg.linear_acceleration.y:.2f}")
            self.accel_z.display(f"{msg.linear_acceleration.z:.2f}")
            
            self.mpu_data['gyro'] = msg.angular_velocity
            self.gyro_x.display(f"{msg.angular_velocity.x:.2f}")
            self.gyro_y.display(f"{msg.angular_velocity.y:.2f}")
            self.gyro_z.display(f"{msg.angular_velocity.z:.2f}")
            
        except Exception as e:
            rospy.logerr(f"IMU callback error: {e}")

    def left_rpm_callback(self, msg):
        try:
            self.motor_data['left_rpm'] = msg.data
            self.left_rpm.display(str(msg.data))
        except Exception as e:
            rospy.logerr(f"Left RPM callback error: {e}")

    def right_rpm_callback(self, msg):
        try:
            self.motor_data['right_rpm'] = msg.data
            self.right_rpm.display(str(msg.data))
        except Exception as e:
            rospy.logerr(f"Right RPM callback error: {e}")

    def left_encoder_callback(self, msg):
        try:
            self.motor_data['left_encoder'] = msg.data
            self.left_encoder.display(str(msg.data))
        except Exception as e:
            rospy.logerr(f"Left encoder callback error: {e}")

    def right_encoder_callback(self, msg):
        try:
            self.motor_data['right_encoder'] = msg.data
            self.right_encoder.display(str(msg.data))
        except Exception as e:
            rospy.logerr(f"Right encoder callback error: {e}")

    def voltage_callback(self, msg):
        try:
            self.system_data['voltage'] = msg.data
        except Exception as e:
            rospy.logerr(f"Voltage callback error: {e}")

    def current_callback(self, msg):
        try:
            self.system_data['current'] = msg.data
        except Exception as e:
            rospy.logerr(f"Current callback error: {e}")

    def temperature_callback(self, msg):
        try:
            self.system_data['temperature'] = msg.data
            self.sys_temp.display(f"{msg.data:.1f}")
            self.imu_temp.display(f"{msg.data:.1f}")
        except Exception as e:
            rospy.logerr(f"Temperature callback error: {e}")

    # Real-time power consumption callbacks with LCD displays
    def rplidar_voltage_callback(self, msg):
        self.rplidar_voltage.display(f"{msg.data:.1f}")
    
    def rplidar_current_callback(self, msg):
        self.rplidar_current.display(f"{msg.data:.2f}")
    
    def rplidar_power_callback(self, msg):
        self.rplidar_power.display(f"{msg.data:.1f}")
    
    def motors_voltage_callback(self, msg):
        self.motors_voltage.display(f"{msg.data:.1f}")
    
    def motors_current_callback(self, msg):
        self.motors_current.display(f"{msg.data:.2f}")
    
    def motors_power_callback(self, msg):
        self.motors_power.display(f"{msg.data:.1f}")
    
    def computing_voltage_callback(self, msg):
        self.computing_voltage.display(f"{msg.data:.1f}")
    
    def computing_current_callback(self, msg):
        self.computing_current.display(f"{msg.data:.2f}")
    
    def computing_power_callback(self, msg):
        self.computing_power.display(f"{msg.data:.1f}")
    
    def total_power_callback(self, msg):
        self.total_power.display(f"{msg.data:.1f}")

    # Real-time mapping metrics callbacks with LCD displays
    def map_resolution_callback(self, msg):
        self.map_resolution.display(f"{msg.data:.1f}")
    
    def coverage_percentage_callback(self, msg):
        self.coverage_percentage.display(f"{msg.data:.1f}")
    
    def wall_consistency_callback(self, msg):
        self.wall_consistency.setText(msg.data)
    
    def feature_accuracy_callback(self, msg):
        self.feature_accuracy.display(f"{msg.data:.1f}")
    
    def mapping_status_callback(self, msg):
        self.mapping_status.setText(msg.data)
        # Change color based on status
        if msg.data.lower() == "active":
            self.mapping_status.setStyleSheet("color: green; background-color: #f0f0f0; padding: 5px; border: 1px solid gray;")
        elif msg.data.lower() == "error":
            self.mapping_status.setStyleSheet("color: red; background-color: #f0f0f0; padding: 5px; border: 1px solid gray;") 
        else:
            self.mapping_status.setStyleSheet("color: blue; background-color: #f0f0f0; padding: 5px; border: 1px solid gray;")
    
    def mapping_progress_callback(self, msg):
        progress_value = int(msg.data)
        self.mapping_progress.setValue(progress_value)

    def map_callback(self, msg):
        """Process actual map data from SLAM for the Map tab"""
        try:
            self.last_map_data = msg
            width = msg.info.width
            height = msg.info.height
            
            # Create image from map data
            image = QImage(width, height, QImage.Format_RGB32)
            
            for y in range(height):
                for x in range(width):
                    index = x + (height - 1 - y) * width
                    if index < len(msg.data):
                        cell_value = msg.data[index]
                        
                        # Map values: -1 = unknown (gray), 0 = free (white), 100 = occupied (black)
                        if cell_value == -1:
                            color = QColor(128, 128, 128)  # Unknown - gray
                        elif cell_value == 0:
                            color = QColor(255, 255, 255)  # Free space - white
                        else:
                            # Occupied space - black to dark gray based on probability
                            intensity = max(0, 255 - int(cell_value * 2.55))
                            color = QColor(intensity, intensity, intensity)
                    else:
                        color = QColor(128, 128, 128)  # Default to unknown
                    
                    image.setPixelColor(x, y, color)
            
            # Store the original image for zoom operations
            self.original_map_image = image.copy()
            
            # Draw robot and markers, then update display
            self.redraw_map_with_markers()
            
        except Exception as e:
            rospy.logerr(f"Map callback error: {e}")
            self.map_label.setText(f"Map Error: {str(e)}")

    def send_goal(self):
        """Handle goal submission with proper validation"""
        try:
            x_text = self.goal_x.toPlainText().strip()
            y_text = self.goal_y.toPlainText().strip()
            theta_text = self.goal_theta.toPlainText().strip()
            
            if not x_text or not y_text or not theta_text:
                raise ValueError("All fields must be filled")
                
            x = float(x_text)
            y = float(y_text)
            theta = float(theta_text)
            
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            
            goal.pose.orientation.z = math.sin(theta / 2.0)
            goal.pose.orientation.w = math.cos(theta / 2.0)
            
            self.goal_pub.publish(goal)
            self.system_messages.append(f"Goal sent: ({x:.2f}, {y:.2f}) θ={theta:.2f} rad")
            self.save_goal_to_history(x, y, theta)
        except ValueError as e:
            self.system_messages.append(f"Error: {str(e)}")
            rospy.logwarn(f"Invalid goal input: {str(e)}")
        except Exception as e:
            self.system_messages.append(f"System error: {str(e)}")
            rospy.logerr(f"Goal submission failed: {str(e)}")

    def update_ui(self):
        """Update UI elements periodically"""
        lin_vel = self.lin_vel_slider.value() / 100.0
        ang_vel = self.ang_vel_slider.value() / 100.0
        self.lin_vel_display.display(f"{lin_vel:.2f}")
        self.ang_vel_display.display(f"{ang_vel:.2f}")

    def global_costmap_callback(self, msg):
        self.process_costmap(msg, self.global_costmap_label, "Global Costmap")
        self.last_global_costmap = msg

    def local_costmap_callback(self, msg):
        self.process_costmap(msg, self.local_costmap_label, "Local Costmap")

    def process_costmap(self, msg, label, title):
        try:
            width = msg.info.width
            height = msg.info.height
            
            image = QImage(width, height, QImage.Format_RGB32)
            
            for y in range(height):
                for x in range(width):
                    index = x + (height - 1 - y) * width
                    if index < len(msg.data):
                        cost = msg.data[index]
                        
                        if cost == -1:
                            color = QColor(100, 100, 100)  # Unknown
                        elif cost == 0:
                            color = QColor(255, 255, 255)  # Free
                        elif cost < 65:
                            color = QColor(0, 255, 0)      # Low cost
                        elif cost < 100:
                            color = QColor(255, 255, 0)    # Medium cost
                        else:
                            color = QColor(255, 0, 0)      # High cost
                    else:
                        color = QColor(100, 100, 100)  # Default to unknown
                    
                    image.setPixelColor(x, y, color)
            
            # Draw robot position marker
            painter = QPainter(image)
            painter.setPen(QColor(0, 0, 255))
            robot_x = width // 2
            robot_y = height // 2
            painter.drawEllipse(robot_x-3, robot_y-3, 6, 6)
            painter.end()
            
            pixmap = QPixmap.fromImage(image)
            label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio))
            
        except Exception as e:
            rospy.logerr(f"Costmap error: {e}")
            label.setText(f"{title} Error")

    def global_plan_callback(self, msg):
        if not hasattr(self, 'global_costmap_label'):
            return
            
        try:
            pixmap = self.global_costmap_label.pixmap()
            if pixmap is None or not hasattr(self, 'last_global_costmap'):
                return
                
            image = pixmap.toImage()
            painter = QPainter(image)
            painter.setPen(QColor(0, 0, 255, 180))
            
            for i in range(len(msg.poses) - 1):
                x1 = int((msg.poses[i].pose.position.x - self.last_global_costmap.info.origin.position.x) / 
                    self.last_global_costmap.info.resolution)
                y1 = int((msg.poses[i].pose.position.y - self.last_global_costmap.info.origin.position.y) / 
                    self.last_global_costmap.info.resolution)
                x2 = int((msg.poses[i+1].pose.position.x - self.last_global_costmap.info.origin.position.x) / 
                    self.last_global_costmap.info.resolution)
                y2 = int((msg.poses[i+1].pose.position.y - self.last_global_costmap.info.origin.position.y) / 
                    self.last_global_costmap.info.resolution)
                
                y1 = self.last_global_costmap.info.height - y1
                y2 = self.last_global_costmap.info.height - y2
                
                painter.drawLine(x1, y1, x2, y2)
            
            painter.end()
            self.global_costmap_label.setPixmap(QPixmap.fromImage(image))
            
        except Exception as e:
            rospy.logerr(f"Path drawing error: {e}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pos_x.setText(f"{msg.pose.pose.position.x:.2f}")
        self.pos_y.setText(f"{msg.pose.pose.position.y:.2f}")
        
        q = msg.pose.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.pos_theta.setText(f"{math.degrees(theta):.1f}°")
        
        # Redraw map with updated robot position
        if hasattr(self, 'original_map_image'):
            self.redraw_map_with_markers()
        
    def sysmsg_callback(self, msg):
        self.system_messages.append(msg.data)
        
    def mode_changed(self, mode):
        self.mode_pub.publish(mode)
        
    def velocity_changed(self):
        lin_vel = self.lin_vel_slider.value() / 100.0
        ang_vel = self.ang_vel_slider.value() / 100.0
        
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel
        self.cmd_vel_pub.publish(twist)
        
    def move_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
    def closeEvent(self, event):
        """Clean up on window close"""
        self.update_timer.stop()
        rospy.signal_shutdown("UI closed")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = RobotControlUI()
    ui.show()
    sys.exit(app.exec_())