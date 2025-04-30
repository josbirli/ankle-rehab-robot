# gui_window.py
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QSlider, QDoubleSpinBox, QComboBox, QTabWidget, QTextEdit,
                             QStatusBar, QGridLayout, QGroupBox, QSizePolicy, QFormLayout)
from PySide6.QtCore import Qt, Slot, Signal
from PySide6.QtGui import QPalette, QColor, QFont
import pyqtgraph as pg
from collections import deque
import time

import config

class MainWindow(QMainWindow):
    # Signals to communicate with worker threads
    set_target_angles_signal = Signal(float, float)
    set_manual_speed_signal = Signal(float)
    set_mode_signal = Signal(str)
    start_control_signal = Signal()
    pause_control_signal = Signal()
    emergency_stop_signal = Signal()
    reset_emergency_stop_signal = Signal()
    set_rom_params_signal = Signal(float, float, float, float, float, int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ankle Rehabilitation Robot Control")
        self.setGeometry(100, 100, 1000, 600) # x, y, width, height

        # --- Data storage for plots ---
        self.time_data = deque(maxlen=config.PLOT_DATA_BUFFER_SIZE)
        self.pd_angle_data = deque(maxlen=config.PLOT_DATA_BUFFER_SIZE)
        self.ie_angle_data = deque(maxlen=config.PLOT_DATA_BUFFER_SIZE)
        self.start_time = time.time()

        # --- Central Widget and Layout ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget) # Main horizontal layout

        # --- Left Panel (Controls & Status) ---
        self.left_panel = QGroupBox("Control & Status")
        self.left_layout = QVBoxLayout()
        self.left_panel.setLayout(self.left_layout)
        self.main_layout.addWidget(self.left_panel, 1) # Weight 1

        # --- Right Panel (Parameters & Data) ---
        self.right_panel = QTabWidget()
        self.main_layout.addWidget(self.right_panel, 3) # Weight 3

        # --- Status Bar ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("Initializing...")
        self.status_bar.addWidget(self.status_label)

        # --- Populate Left Panel ---
        self._create_left_panel()

        # --- Populate Right Panel ---
        self._create_right_panel()

        # --- Apply some styling ---
        self._apply_styles()

        # --- Internal state ---
        self._is_running = False
        self._is_estopped = False

    def _create_left_panel(self):
        # Emergency Stop Button
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setFixedHeight(50)
        self.emergency_stop_button.clicked.connect(self.on_emergency_stop)
        self.left_layout.addWidget(self.emergency_stop_button)

        # Reset E-Stop Button (Initially hidden)
        self.reset_estop_button = QPushButton("Reset E-Stop")
        self.reset_estop_button.clicked.connect(self.on_reset_estop)
        self.reset_estop_button.setVisible(False)
        self.left_layout.addWidget(self.reset_estop_button)


        # Mode Selection
        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout()
        mode_group.setLayout(mode_layout)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([config.MODE_IDLE, config.MODE_MANUAL, config.MODE_PASSIVE_ROM, config.MODE_CALIBRATING])
        self.mode_combo.currentTextChanged.connect(self.on_mode_change)
        mode_layout.addWidget(self.mode_combo)
        self.left_layout.addWidget(mode_group)

        # Start/Pause Button
        self.start_pause_button = QPushButton("Start")
        self.start_pause_button.setFixedHeight(40)
        self.start_pause_button.clicked.connect(self.on_start_pause)
        self.start_pause_button.setEnabled(False) # Disabled until mode selected != Idle
        self.left_layout.addWidget(self.start_pause_button)

        # Status Indicators
        status_group = QGroupBox("Status")
        status_layout = QFormLayout()
        status_group.setLayout(status_layout)

        self.robot_status_label = QLabel(config.MODE_IDLE)
        self.sensor_status_label = QLabel("Sensors OK")
        self.pd_angle_label = QLabel("PD: --- °")
        self.ie_angle_label = QLabel("IE: --- °")

        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.pd_angle_label.setFont(font)
        self.ie_angle_label.setFont(font)


        status_layout.addRow("Robot:", self.robot_status_label)
        status_layout.addRow("Sensors:", self.sensor_status_label)
        status_layout.addRow(self.pd_angle_label)
        status_layout.addRow(self.ie_angle_label)
        self.left_layout.addWidget(status_group)

        self.left_layout.addStretch(1) # Push elements to top

    def _create_right_panel(self):
        # --- Tab 1: Manual Control ---
        self.manual_tab = QWidget()
        manual_layout = QVBoxLayout(self.manual_tab)

        # PD Control Group
        pd_group = QGroupBox("Plantarflexion / Dorsiflexion (PD)")
        pd_layout = QGridLayout()
        pd_group.setLayout(pd_layout)
        self.pd_slider = QSlider(Qt.Horizontal)
        self.pd_slider.setRange(int(config.MIN_PD_ANGLE * 10), int(config.MAX_PD_ANGLE * 10)) # Use 10x for precision
        self.pd_slider.setValue(0)
        self.pd_slider.valueChanged.connect(self.on_pd_slider_change)
        self.pd_spinbox = QDoubleSpinBox()
        self.pd_spinbox.setRange(config.MIN_PD_ANGLE, config.MAX_PD_ANGLE)
        self.pd_spinbox.setSingleStep(0.5)
        self.pd_spinbox.setSuffix(" °")
        self.pd_spinbox.setValue(0.0)
        self.pd_spinbox.valueChanged.connect(self.on_pd_spinbox_change)
        pd_layout.addWidget(QLabel("Target Angle:"), 0, 0)
        pd_layout.addWidget(self.pd_slider, 1, 0, 1, 2)
        pd_layout.addWidget(self.pd_spinbox, 0, 1)
        manual_layout.addWidget(pd_group)

        # IE Control Group
        ie_group = QGroupBox("Inversion / Eversion (IE)")
        ie_layout = QGridLayout()
        ie_group.setLayout(ie_layout)
        self.ie_slider = QSlider(Qt.Horizontal)
        self.ie_slider.setRange(int(config.MIN_IE_ANGLE * 10), int(config.MAX_IE_ANGLE * 10))
        self.ie_slider.setValue(0)
        self.ie_slider.valueChanged.connect(self.on_ie_slider_change)
        self.ie_spinbox = QDoubleSpinBox()
        self.ie_spinbox.setRange(config.MIN_IE_ANGLE, config.MAX_IE_ANGLE)
        self.ie_spinbox.setSingleStep(0.5)
        self.ie_spinbox.setSuffix(" °")
        self.ie_spinbox.setValue(0.0)
        self.ie_spinbox.valueChanged.connect(self.on_ie_spinbox_change)
        ie_layout.addWidget(QLabel("Target Angle:"), 0, 0)
        ie_layout.addWidget(self.ie_slider, 1, 0, 1, 2)
        ie_layout.addWidget(self.ie_spinbox, 0, 1)
        manual_layout.addWidget(ie_group)

        # Speed Control Group
        speed_group = QGroupBox("Manual Speed Control")
        speed_layout = QHBoxLayout()
        speed_group.setLayout(speed_layout)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100) # Percentage
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_slider_change)
        self.speed_label = QLabel("50 %")
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        manual_layout.addWidget(speed_group)

        manual_layout.addStretch(1)
        self.right_panel.addTab(self.manual_tab, "Manual Control")

        # --- Tab 2: Exercise Setup ---
        self.exercise_tab = QWidget()
        exercise_layout = QVBoxLayout(self.exercise_tab)

        rom_group = QGroupBox("Passive Range of Motion (ROM) Setup")
        rom_layout = QFormLayout()
        rom_group.setLayout(rom_layout)

        self.rom_max_pd_spin = QDoubleSpinBox(suffix=" °")
        self.rom_max_pd_spin.setRange(0, config.MAX_PD_ANGLE)
        self.rom_max_pd_spin.setValue(config.MAX_PD_ANGLE / 2)
        self.rom_min_pd_spin = QDoubleSpinBox(suffix=" °")
        self.rom_min_pd_spin.setRange(config.MIN_PD_ANGLE, 0)
        self.rom_min_pd_spin.setValue(config.MIN_PD_ANGLE / 2)

        self.rom_max_ie_spin = QDoubleSpinBox(suffix=" °")
        self.rom_max_ie_spin.setRange(0, config.MAX_IE_ANGLE)
        self.rom_max_ie_spin.setValue(config.MAX_IE_ANGLE / 2)
        self.rom_min_ie_spin = QDoubleSpinBox(suffix=" °")
        self.rom_min_ie_spin.setRange(config.MIN_IE_ANGLE, 0)
        self.rom_min_ie_spin.setValue(config.MIN_IE_ANGLE / 2)

        self.rom_speed_spin = QDoubleSpinBox(suffix=" °/s")
        self.rom_speed_spin.setRange(1.0, config.MAX_SPEED_DPS)
        self.rom_speed_spin.setValue(10.0)

        self.rom_reps_spin = QDoubleSpinBox() # Use DoubleSpinBox for consistency, treat as int later
        self.rom_reps_spin.setRange(1, 100)
        self.rom_reps_spin.setDecimals(0)
        self.rom_reps_spin.setValue(5)


        rom_layout.addRow("Max Plantarflexion:", self.rom_max_pd_spin)
        rom_layout.addRow("Max Dorsiflexion:", self.rom_min_pd_spin)
        rom_layout.addRow("Max Inversion:", self.rom_max_ie_spin)
        rom_layout.addRow("Max Eversion:", self.rom_min_ie_spin)
        rom_layout.addRow("Movement Speed:", self.rom_speed_spin)
        rom_layout.addRow("Repetitions:", self.rom_reps_spin)

        # Apply Button
        self.apply_rom_button = QPushButton("Apply ROM Settings")
        self.apply_rom_button.clicked.connect(self.on_apply_rom_settings)
        rom_layout.addRow(self.apply_rom_button)


        exercise_layout.addWidget(rom_group)
        exercise_layout.addStretch(1)
        self.right_panel.addTab(self.exercise_tab, "Exercise Setup")


        # --- Tab 3: Real-time Plots ---
        self.plot_tab = QWidget()
        plot_layout = QVBoxLayout(self.plot_tab)

        # Use pyqtgraph for plotting
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.pd_plot_widget = pg.PlotWidget(title="Plantar/Dorsiflexion Angle")
        self.pd_plot_widget.setLabel('left', 'Angle (°)')
        self.pd_plot_widget.setLabel('bottom', 'Time (s)')
        self.pd_plot_widget.showGrid(x=True, y=True)
        self.pd_curve_actual = self.pd_plot_widget.plot(pen='b', name='Actual')
        # self.pd_curve_target = self.pd_plot_widget.plot(pen='r', style=Qt.DashLine, name='Target') # Optional: Plot target
        self.pd_plot_widget.addLegend()

        self.ie_plot_widget = pg.PlotWidget(title="Inversion/Eversion Angle")
        self.ie_plot_widget.setLabel('left', 'Angle (°)')
        self.ie_plot_widget.setLabel('bottom', 'Time (s)')
        self.ie_plot_widget.showGrid(x=True, y=True)
        self.ie_curve_actual = self.ie_plot_widget.plot(pen='g', name='Actual')
        # self.ie_curve_target = self.ie_plot_widget.plot(pen='r', style=Qt.DashLine, name='Target') # Optional
        self.ie_plot_widget.addLegend()


        plot_layout.addWidget(self.pd_plot_widget)
        plot_layout.addWidget(self.ie_plot_widget)
        self.right_panel.addTab(self.plot_tab, "Real-time Plots")

        # --- Tab 4: Session Log ---
        self.log_tab = QWidget()
        log_layout = QVBoxLayout(self.log_tab)
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        log_layout.addWidget(self.log_text_edit)
        self.right_panel.addTab(self.log_tab, "Session Log")

        # Initially disable tabs/widgets that require specific modes
        self.manual_tab.setEnabled(False)
        self.exercise_tab.setEnabled(False)


    def _apply_styles(self):
        self.emergency_stop_button.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid darkred;
                border-radius: 5px;
            }
            QPushButton:pressed {
                background-color: darkred;
            }
        """)
        self.reset_estop_button.setStyleSheet("""
            QPushButton {
                background-color: orange;
                color: black;
                font-size: 14px;
                border-radius: 5px;
            }
             QPushButton:pressed {
                background-color: darkorange;
            }
        """)

        self.start_pause_button.setStyleSheet("""
            QPushButton {
                background-color: lightgreen;
                font-size: 14px;
                font-weight: bold;
                 border-radius: 5px;
            }
             QPushButton:pressed {
                background-color: green;
            }
            QPushButton:!enabled {
                background-color: lightgray;
            }
        """)
        # You can add more styling using QSS (Qt Style Sheets) for a cleaner look

    # --- SLOTS for UI interactions ---
    @Slot()
    def on_emergency_stop(self):
        self.emergency_stop_signal.emit()
        self._is_estopped = True
        self.emergency_stop_button.setEnabled(False) # Disable after pressed
        self.reset_estop_button.setVisible(True)
        self.start_pause_button.setEnabled(False)
        self.start_pause_button.setText("Start")
        self._is_running = False
        # Disable controls
        self.manual_tab.setEnabled(False)
        # Don't disable exercise tab, might need to see settings
        # self.exercise_tab.setEnabled(False)
        self.mode_combo.setEnabled(False) # Prevent mode changes during E-Stop

    @Slot()
    def on_reset_estop(self):
        self.reset_emergency_stop_signal.emit()
        self._is_estopped = False
        self.reset_estop_button.setVisible(False)
        self.emergency_stop_button.setEnabled(True)
        self.mode_combo.setEnabled(True)
        self.mode_combo.setCurrentIndex(0) # Force back to Idle mode
        self.on_mode_change(config.MODE_IDLE) # Trigger mode update logic


    @Slot()
    def on_start_pause(self):
        if self._is_estopped: return # Should not happen if button disabled, but check anyway

        if not self._is_running:
            self.start_control_signal.emit()
            self.start_pause_button.setText("Pause")
            self.start_pause_button.setStyleSheet("background-color: yellow; font-size: 14px; font-weight: bold; border-radius: 5px;")
            self._is_running = True
            self.mode_combo.setEnabled(False) # Prevent mode change while running
            # Disable exercise settings while running
            if self.mode_combo.currentText() == config.MODE_PASSIVE_ROM:
                 self.exercise_tab.setEnabled(False)
        else:
            self.pause_control_signal.emit()
            self.start_pause_button.setText("Start")
            self.start_pause_button.setStyleSheet("background-color: lightgreen; font-size: 14px; font-weight: bold; border-radius: 5px;")
            self._is_running = False
            self.mode_combo.setEnabled(True) # Allow mode change when paused
            # Re-enable exercise settings when paused
            if self.mode_combo.currentText() == config.MODE_PASSIVE_ROM:
                 self.exercise_tab.setEnabled(True)


    @Slot(str)
    def on_mode_change(self, mode):
        if self._is_estopped: return # Don't allow mode change during E-Stop

        self.set_mode_signal.emit(mode)
        # Update UI based on mode
        self.manual_tab.setEnabled(mode == config.MODE_MANUAL)
        self.exercise_tab.setEnabled(mode == config.MODE_PASSIVE_ROM)

        # Enable/disable start button
        can_start = mode != config.MODE_IDLE and mode != config.MODE_CALIBRATING
        self.start_pause_button.setEnabled(can_start)

        # Reset start/pause button state if changing mode
        if self._is_running:
             self.pause_control_signal.emit() # Pause if running
             self.start_pause_button.setText("Start")
             self.start_pause_button.setStyleSheet("background-color: lightgreen; font-size: 14px; font-weight: bold; border-radius: 5px;")
             self._is_running = False

        self.add_log_entry(f"Mode changed to: {mode}")


    @Slot(int)
    def on_pd_slider_change(self, value):
        angle = value / 10.0
        # Update spinbox without triggering its own signal loop
        self.pd_spinbox.blockSignals(True)
        self.pd_spinbox.setValue(angle)
        self.pd_spinbox.blockSignals(False)
        self.set_target_angles_signal.emit(angle, self.ie_spinbox.value())

    @Slot(float)
    def on_pd_spinbox_change(self, value):
        # Update slider without triggering its own signal loop
        self.pd_slider.blockSignals(True)
        self.pd_slider.setValue(int(value * 10))
        self.pd_slider.blockSignals(False)
        self.set_target_angles_signal.emit(value, self.ie_spinbox.value())

    @Slot(int)
    def on_ie_slider_change(self, value):
        angle = value / 10.0
        self.ie_spinbox.blockSignals(True)
        self.ie_spinbox.setValue(angle)
        self.ie_spinbox.blockSignals(False)
        self.set_target_angles_signal.emit(self.pd_spinbox.value(), angle)

    @Slot(float)
    def on_ie_spinbox_change(self, value):
        self.ie_slider.blockSignals(True)
        self.ie_slider.setValue(int(value * 10))
        self.ie_slider.blockSignals(False)
        self.set_target_angles_signal.emit(self.pd_spinbox.value(), value)

    @Slot(int)
    def on_speed_slider_change(self, value):
        self.speed_label.setText(f"{value} %")
        self.set_manual_speed_signal.emit(float(value))

    @Slot()
    def on_apply_rom_settings(self):
         min_pd = self.rom_min_pd_spin.value()
         max_pd = self.rom_max_pd_spin.value()
         min_ie = self.rom_min_ie_spin.value()
         max_ie = self.rom_max_ie_spin.value()
         speed = self.rom_speed_spin.value()
         reps = int(self.rom_reps_spin.value()) # Ensure integer

         # Add basic validation
         if min_pd >= max_pd or min_ie >= max_ie:
              self.add_log_entry("Error: Min angle cannot be greater than or equal to Max angle in ROM settings.")
              # Optionally show a message box
              return

         self.set_rom_params_signal.emit(min_pd, max_pd, min_ie, max_ie, speed, reps)
         self.add_log_entry("Applied new ROM settings.")


    # --- SLOTS for receiving updates from worker threads ---
    @Slot(float, float)
    def update_angle_display(self, pd_angle, ie_angle):
        self.pd_angle_label.setText(f"PD: {pd_angle:+.1f} °")
        self.ie_angle_label.setText(f"IE: {ie_angle:+.1f} °")

        # Update plots
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.pd_angle_data.append(pd_angle)
        self.ie_angle_data.append(ie_angle)

        self.pd_curve_actual.setData(list(self.time_data), list(self.pd_angle_data))
        self.ie_curve_actual.setData(list(self.time_data), list(self.ie_angle_data))

    @Slot(str)
    def update_status_label(self, status):
        self.robot_status_label.setText(status)
        self.status_label.setText(f"Status: {status}") # Update status bar too

    @Slot(str)
    def update_sensor_status(self, status):
        self.sensor_status_label.setText(status)
        if "error" in status.lower() or "fail" in status.lower():
             self.sensor_status_label.setStyleSheet("color: red;")
        else:
             self.sensor_status_label.setStyleSheet("color: green;")

    @Slot(str)
    def add_log_entry(self, message):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text_edit.append(f"[{timestamp}] {message}")

    def closeEvent(self, event):
        """Ensure threads are stopped cleanly on window close."""
        self.add_log_entry("Shutdown requested...")
        # Signal threads to stop (implement stop methods in threads)
        # Wait for threads to finish (implement wait in main script)
        event.accept() # Allow window to close