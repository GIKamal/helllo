#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QTabWidget, QGroupBox, QLabel, QPushButton, QComboBox, 
                            QSlider, QLCDNumber, QTextEdit, QScrollArea)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor
import rospy # type: ignore
from std_msgs.msg import String, Bool, Float32 # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore
from nav_msgs.msg import Odometry, OccupancyGrid, Path # type: ignore
from geometry_msgs.msg import Twist, PoseStamped # type: ignore
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
        self.setGeometry(100, 100, 1200, 800)
        
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
        self.main_layout.addWidget(self.left_panel, 30)
        self.main_layout.addWidget(self.right_panel, 70)
        
        # Initialize UI components
        self.init_control_panel()
        self.init_status_panel()
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
        self.btn_forward = QPushButton("� Forward")
        self.btn_backward = QPushButton("� Backward")
        button_row1.addWidget(self.btn_forward)
        button_row1.addWidget(self.btn_backward)
        
        button_row2 = QHBoxLayout()
        self.btn_left = QPushButton("� Left")
        self.btn_stop = QPushButton("� Stop")
        self.btn_right = QPushButton("� Right")
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
        theta_layout.addWidget(QLabel("�:"))
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
        goal_str = f"X: {x:.2f}, Y: {y:.2f}, �: {theta:.2f}"
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

    def init_status_panel(self):
        """Initialize the status monitoring panel"""
        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()
        
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(QLabel("Battery:"))
        self.battery_level = QLCDNumber()
        self.battery_level.setDigitCount(5)
        self.battery_level.display("---")
        battery_layout.addWidget(self.battery_level)
        
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("ROS:"))
        self.ros_status = QLabel("Disconnected")
        self.ros_status.setStyleSheet("color: red;")
        conn_layout.addWidget(self.ros_status)
        
        pos_group = QGroupBox("Position")
        pos_layout = QVBoxLayout()
        
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X:"))
        self.pos_x = QLabel("0.00")
        x_layout.addWidget(self.pos_x)
        
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y:"))
        self.pos_y = QLabel("0.00")
        y_layout.addWidget(self.pos_y)
        
        theta_layout = QHBoxLayout()
        theta_layout.addWidget(QLabel("�:"))
        self.pos_theta = QLabel("0.00")
        theta_layout.addWidget(self.pos_theta)
        
        pos_layout.addLayout(x_layout)
        pos_layout.addLayout(y_layout)
        pos_layout.addLayout(theta_layout)
        pos_group.setLayout(pos_layout)
        
        sys_msg_group = QGroupBox("System Messages")
        sys_msg_layout = QVBoxLayout()
        self.system_messages = QTextEdit()
        self.system_messages.setReadOnly(True)
        sys_msg_layout.addWidget(self.system_messages)
        sys_msg_group.setLayout(sys_msg_layout)
        
        status_layout.addLayout(battery_layout)
        status_layout.addLayout(conn_layout)
        status_layout.addWidget(pos_group)
        status_layout.addWidget(sys_msg_group)
        status_group.setLayout(status_layout)
        
        self.left_layout.addWidget(status_group)

    def init_visualization_panel(self):
        """Initialize the visualization panel with sensor data and RViz"""
        vis_tabs = QTabWidget()
        
        # Tab 1: LIDAR Visualization
        lidar_tab = QWidget()
        lidar_layout = QVBoxLayout(lidar_tab)
        
        self.lidar_label = QLabel()
        self.lidar_label.setAlignment(Qt.AlignCenter)
        self.lidar_label.setMinimumSize(640, 480)
        self.lidar_label.setText("LIDAR Visualization")
        lidar_layout.addWidget(self.lidar_label)
        
        # Tab 2: Map View with Zoom Controls
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
        
        # Tab 3: RViz Visualization
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
        vis_tabs.addTab(lidar_tab, "LIDAR")
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
        
        # FIXED: Added map subscriber for the Map tab
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        rospy.Subscriber('/system_messages', String, self.sysmsg_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_callback)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.global_plan_callback)
        
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
            self.system_messages.append(f"Goal sent: ({x:.2f}, {y:.2f}) �={theta:.2f} rad")
            self.save_goal_to_history(x, y, theta)
        except ValueError as e:
            self.system_messages.append(f"Error: {str(e)}")
            rospy.logwarn(f"Invalid goal input: {str(e)}")
        except Exception as e:
            self.system_messages.append(f"System error: {str(e)}")
            rospy.logerr(f"Goal submission failed: {str(e)}")

    def update_ui(self):
        """Update UI elements periodically"""
        if rospy.is_shutdown():
            self.ros_status.setText("Disconnected")
            self.ros_status.setStyleSheet("color: red;")
        else:
            self.ros_status.setText("Connected")
            self.ros_status.setStyleSheet("color: green;")
            
        lin_vel = self.lin_vel_slider.value() / 100.0
        ang_vel = self.ang_vel_slider.value() / 100.0
        self.lin_vel_display.display(f"{lin_vel:.2f}")
        self.ang_vel_display.display(f"{ang_vel:.2f}")

    def lidar_callback(self, msg):
        try:
            image = QImage(640, 480, QImage.Format_RGB32)
            image.fill(Qt.black)
            painter = QPainter(image)
            painter.setPen(Qt.green)
            
            center_x, center_y = 320, 240
            max_range = 5.0
            scale = 200 / max_range
            
            for i, distance in enumerate(msg.ranges):
                if math.isinf(distance) or math.isnan(distance):
                    continue
                    
                angle = msg.angle_min + i * msg.angle_increment
                x = center_x + distance * scale * math.cos(angle)
                y = center_y - distance * scale * math.sin(angle)  # Flip Y for correct orientation
                painter.drawPoint(int(x), int(y))
                
            painter.end()
            pixmap = QPixmap.fromImage(image)
            self.lidar_label.setPixmap(pixmap.scaled(self.lidar_label.size(), Qt.KeepAspectRatio))
        except Exception as e:
            rospy.logerr(f"LIDAR callback error: {e}")

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
        self.pos_theta.setText(f"{math.degrees(theta):.1f}�")
        
        # Redraw map with updated robot position
        if hasattr(self, 'original_map_image'):
            self.redraw_map_with_markers()
        
    def battery_callback(self, msg):
        self.battery_level.display(f"{msg.data:.1f}%")
        
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