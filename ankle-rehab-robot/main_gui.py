# main_gui.py
import sys
import time
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer # For periodic checks if needed

import hardware_interface as hw
from gui_window import MainWindow
from worker_threads import SensorThread, ControlThread
import config

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # --- Initialize Hardware ---
    try:
        hw.setup_gpio()
    except Exception as e:
        print(f"FATAL: Failed to initialize hardware: {e}")
        # Optionally show a critical error message box to the user
        # For now, we'll allow GUI to load but hardware interaction will fail
        # In a real scenario, might exit here.
        pass # Allow GUI to load to show error

    # --- Create GUI Window ---
    main_window = MainWindow()

    # --- Create Worker Threads ---
    sensor_thread = SensorThread()
    control_thread = ControlThread()

    # --- Connect Signals and Slots ---

    # Sensor thread outputs -> GUI updates
    sensor_thread.angles_updated.connect(main_window.update_angle_display)
    sensor_thread.angles_updated.connect(control_thread.update_current_angles) # Sensor -> Control Thread
    sensor_thread.sensor_error.connect(main_window.update_sensor_status)
    sensor_thread.sensor_error.connect(main_window.add_log_entry)
    # Optional: Connect sensor_error to control_thread emergency stop?
    # sensor_thread.sensor_error.connect(control_thread.emergency_stop)

    # Control thread outputs -> GUI updates
    control_thread.status_update.connect(main_window.update_status_label)
    control_thread.log_message.connect(main_window.add_log_entry)
    control_thread.control_error.connect(main_window.add_log_entry)
    control_thread.control_error.connect(lambda msg: main_window.update_status_label(f"ERROR: {msg}")) # Show error in status
    # Connect control error to potentially trigger GUI E-Stop state visual
    control_thread.control_error.connect(main_window.on_emergency_stop)


    # GUI actions -> Control thread commands
    main_window.set_target_angles_signal.connect(control_thread.set_target_angles)
    main_window.set_manual_speed_signal.connect(control_thread.set_manual_speed)
    main_window.set_mode_signal.connect(control_thread.set_mode)
    main_window.start_control_signal.connect(control_thread.start_control)
    main_window.pause_control_signal.connect(control_thread.pause_control)
    main_window.emergency_stop_signal.connect(control_thread.emergency_stop)
    main_window.reset_emergency_stop_signal.connect(control_thread.reset_emergency_stop)
    main_window.set_rom_params_signal.connect(control_thread.set_rom_params)


    # --- Start Threads ---
    try:
        sensor_thread.start()
        control_thread.start()
    except Exception as e:
         print(f"FATAL: Failed to start threads: {e}")
         main_window.add_log_entry(f"FATAL ERROR starting threads: {e}")
         # Consider exiting or disabling controls


    # --- Show GUI ---
    main_window.show()
    main_window.add_log_entry("System Initialized. Ready.")


    # --- Start Event Loop ---
    exit_code = app.exec()

    # --- Cleanup ---
    print("Application closing...")
    main_window.add_log_entry("Shutting down...")

    # Stop threads gracefully
    control_thread.stop()
    sensor_thread.stop()

    # Wait for threads to finish
    print("Waiting for ControlThread to finish...")
    control_thread.wait(5000) # Wait max 5 seconds
    if control_thread.isRunning():
        print("Warning: ControlThread did not terminate gracefully.")
        control_thread.terminate() # Force terminate if stuck

    print("Waiting for SensorThread to finish...")
    sensor_thread.wait(5000) # Wait max 5 seconds
    if sensor_thread.isRunning():
         print("Warning: SensorThread did not terminate gracefully.")
         sensor_thread.terminate() # Force terminate if stuck

    # Cleanup hardware
    hw.cleanup_gpio()

    print("Cleanup complete. Exiting.")
    sys.exit(exit_code)