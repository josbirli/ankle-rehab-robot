# worker_threads.py
import time
from PySide6.QtCore import QThread, Signal, QMutex, QWaitCondition
from simple_pid import PID

import hardware_interface as hw
import config

class SensorThread(QThread):
    """Reads sensor data periodically."""
    angles_updated = Signal(float, float) # pd_angle, ie_angle
    sensor_error = Signal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._mutex = QMutex()

    def run(self):
        self._mutex.lock()
        self._running = True
        self._mutex.unlock()

        while True:
            self._mutex.lock()
            if not self._running:
                self._mutex.unlock()
                break
            self._mutex.unlock()

            try:
                pd_angle = hw.read_sensor(config.AXIS_PD)
                ie_angle = hw.read_sensor(config.AXIS_IE)

                if pd_angle is not None and ie_angle is not None:
                    self.angles_updated.emit(pd_angle, ie_angle)
                else:
                    if pd_angle is None:
                        self.sensor_error.emit("Failed to read PD sensor.")
                    if ie_angle is None:
                        self.sensor_error.emit("Failed to read IE sensor.")
                    # Optional: Pause or stop control thread on sensor error?

            except Exception as e:
                self.sensor_error.emit(f"Sensor thread error: {e}")
                # Consider stopping loop on critical errors

            time.sleep(config.SENSOR_READ_INTERVAL) # Control loop frequency

        print("SensorThread finished.")

    def stop(self):
        self._mutex.lock()
        self._running = False
        self._mutex.unlock()
        print("Stopping SensorThread...")


class ControlThread(QThread):
    """Handles motor control logic, PID, and safety checks."""
    status_update = Signal(str)
    log_message = Signal(str)
    control_error = Signal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._paused = True
        self._emergency_stopped = False
        self._mutex = QMutex()
        self._wait_condition = QWaitCondition() # To pause thread efficiently

        self.current_mode = config.MODE_IDLE
        self.target_pd_angle = 0.0
        self.target_ie_angle = 0.0
        self.current_pd_angle = 0.0
        self.current_ie_angle = 0.0
        self.manual_speed_factor = 0.5 # 0.0 to 1.0 for manual slider speed

        # PID Controllers
        self.pd_pid = PID(config.PD_PID_KP, config.PD_PID_KI, config.PD_PID_KD, setpoint=0)
        self.ie_pid = PID(config.IE_PID_KP, config.IE_PID_KI, config.IE_PID_KD, setpoint=0)
        self.pd_pid.output_limits = (-config.MAX_PWM_OUTPUT, config.MAX_PWM_OUTPUT)
        self.ie_pid.output_limits = (-config.MAX_PWM_OUTPUT, config.MAX_PWM_OUTPUT)
        self.pd_pid.sample_time = config.CONTROL_LOOP_INTERVAL
        self.ie_pid.sample_time = config.CONTROL_LOOP_INTERVAL

        # Exercise parameters (example for Passive ROM)
        self.rom_min_pd = config.MIN_PD_ANGLE
        self.rom_max_pd = config.MAX_PD_ANGLE
        self.rom_min_ie = config.MIN_IE_ANGLE
        self.rom_max_ie = config.MAX_IE_ANGLE
        self.rom_speed_dps = 10.0 # Degrees per second for ROM
        self.rom_reps = 5
        self._rom_current_rep = 0
        self._rom_state = "idle" # e.g., "to_max_pd", "to_min_pd", "to_max_ie", ...

    def update_current_angles(self, pd_angle, ie_angle):
        """Slot to receive angle updates from SensorThread."""
        self._mutex.lock()
        self.current_pd_angle = pd_angle
        self.current_ie_angle = ie_angle
        self._mutex.unlock()

    def set_target_angles(self, pd_target, ie_target):
        self._mutex.lock()
        self.target_pd_angle = pd_target
        self.target_ie_angle = ie_target
        # Update PID setpoints only if in manual mode? Or always? Let's update always for now.
        self.pd_pid.setpoint = self.target_pd_angle
        self.ie_pid.setpoint = self.target_ie_angle
        # print(f"ControlThread: New targets PD={pd_target:.1f}, IE={ie_target:.1f}")
        self._mutex.unlock()

    def set_manual_speed(self, speed_percentage):
        self._mutex.lock()
        # Ensure speed is between 0 and 100
        speed_percentage = max(0.0, min(100.0, speed_percentage))
        self.manual_speed_factor = speed_percentage / 100.0
        self._mutex.unlock()

    def set_mode(self, mode):
        self._mutex.lock()
        if mode != self.current_mode:
            self.log_message.emit(f"Changing mode to: {mode}")
            self.current_mode = mode
            self._paused = True # Pause when changing mode
            self._reset_exercise_state()
            # Reset PID controllers when changing mode to avoid integral windup issues?
            self.pd_pid.reset()
            self.ie_pid.reset()
            hw.set_motor_pwm(config.AXIS_PD, 0) # Ensure motors stop on mode change
            hw.set_motor_pwm(config.AXIS_IE, 0)
        self._mutex.unlock()

    def set_rom_params(self, min_pd, max_pd, min_ie, max_ie, speed, reps):
        self._mutex.lock()
        self.rom_min_pd = min_pd
        self.rom_max_pd = max_pd
        self.rom_min_ie = min_ie
        self.rom_max_ie = max_ie
        self.rom_speed_dps = max(1.0, speed) # Ensure minimum speed
        self.rom_reps = reps
        self.log_message.emit(f"Set ROM Params: PD[{min_pd:.1f},{max_pd:.1f}], IE[{min_ie:.1f},{max_ie:.1f}], Speed:{speed:.1f}, Reps:{reps}")
        self._mutex.unlock()

    def _reset_exercise_state(self):
        self._rom_current_rep = 0
        self._rom_state = "idle"
        # Reset targets to current position to avoid jumps? Or to zero? Let's use zero.
        self.target_pd_angle = 0.0
        self.target_ie_angle = 0.0
        self.pd_pid.setpoint = 0.0
        self.ie_pid.setpoint = 0.0


    def start_control(self):
        self._mutex.lock()
        if not self._emergency_stopped:
            self._paused = False
            self.log_message.emit("Control started.")
            self.status_update.emit(f"Running: {self.current_mode}")
            self._wait_condition.wakeAll() # Wake up if paused
        else:
            self.log_message.emit("Cannot start, E-Stop is active.")
        self._mutex.unlock()

    def pause_control(self):
        self._mutex.lock()
        self._paused = True
        # Stop motors immediately on pause
        hw.set_motor_pwm(config.AXIS_PD, 0)
        hw.set_motor_pwm(config.AXIS_IE, 0)
        self.log_message.emit("Control paused.")
        self.status_update.emit(f"Paused: {self.current_mode}")
        self._mutex.unlock()

    def emergency_stop(self):
        self._mutex.lock()
        self._emergency_stopped = True
        self._paused = True # Ensure it's paused
        # !!! CRITICAL: Stop motors immediately !!!
        hw.set_motor_pwm(config.AXIS_PD, 0)
        hw.set_motor_pwm(config.AXIS_IE, 0)
        self.status_update.emit(config.MODE_ESTOP)
        self.log_message.emit("--- EMERGENCY STOP ACTIVATED ---")
        self._mutex.unlock()

    def reset_emergency_stop(self):
        self._mutex.lock()
        if self._emergency_stopped:
            self._emergency_stopped = False
            self._paused = True # Remain paused until explicitly started
            self.current_mode = config.MODE_IDLE # Go to idle after E-Stop reset
            self._reset_exercise_state()
            self.log_message.emit("Emergency stop reset. System Idle and Paused.")
            self.status_update.emit(config.MODE_IDLE)
        self._mutex.unlock()

    def run(self):
        self._mutex.lock()
        self._running = True
        self._paused = True # Start paused
        self._emergency_stopped = False
        self.current_mode = config.MODE_IDLE
        self.status_update.emit(config.MODE_IDLE)
        self._mutex.unlock()

        while True:
            self._mutex.lock()
            if not self._running:
                self._mutex.unlock()
                break # Exit thread loop

            # --- Emergency Stop Check ---
            if self._emergency_stopped:
                # Ensure motors stay off
                hw.set_motor_pwm(config.AXIS_PD, 0)
                hw.set_motor_pwm(config.AXIS_IE, 0)
                self._mutex.unlock()
                time.sleep(0.1) # Sleep longer during E-Stop
                continue

            # --- Pause Check ---
            if self._paused:
                # Wait until woken up by start_control or mode change
                self._wait_condition.wait(self._mutex)
                # Re-check running flag after wake-up
                if not self._running:
                    self._mutex.unlock()
                    break
                # Don't process commands immediately after waking, loop again
                self._mutex.unlock()
                continue

            # --- Main Control Logic ---
            mode = self.current_mode
            pd_angle = self.current_pd_angle
            ie_angle = self.current_ie_angle
            target_pd = self.target_pd_angle
            target_ie = self.target_ie_angle
            speed_factor = self.manual_speed_factor

            # --- Software Limit Check ---
            safe_pd_target = max(config.MIN_PD_ANGLE, min(config.MAX_PD_ANGLE, target_pd))
            safe_ie_target = max(config.MIN_IE_ANGLE, min(config.MAX_IE_ANGLE, target_ie))
            if safe_pd_target != target_pd:
                self.log_message.emit(f"Warning: PD Target ({target_pd:.1f}) outside limits. Clamped to {safe_pd_target:.1f}")
                self.target_pd_angle = safe_pd_target # Update internal target
                self.pd_pid.setpoint = safe_pd_target
            if safe_ie_target != target_ie:
                self.log_message.emit(f"Warning: IE Target ({target_ie:.1f}) outside limits. Clamped to {safe_ie_target:.1f}")
                self.target_ie_angle = safe_ie_target
                self.ie_pid.setpoint = safe_ie_target


            pd_output = 0.0
            ie_output = 0.0

            try:
                # --- Mode-Specific Logic ---
                if mode == config.MODE_MANUAL:
                    # Calculate PID output based on current vs target
                    pd_output = self.pd_pid(pd_angle)
                    ie_output = self.ie_pid(ie_angle)
                    # Scale PID output by manual speed factor
                    pd_output *= speed_factor
                    ie_output *= speed_factor

                elif mode == config.MODE_PASSIVE_ROM:
                    # --- Simple Passive ROM state machine ---
                    # This needs refinement (smooth transitions, timing, proper state handling)
                    delta_limit = 1.0 # Degrees close enough to target

                    if self._rom_state == "idle":
                       self.log_message.emit("Starting Passive ROM exercise.")
                       self._rom_current_rep = 1
                       self._rom_state = "to_max_pd" # Start with PD movement
                       self.log_message.emit(f"Rep {self._rom_current_rep}/{self.rom_reps}")

                    # Determine target for current state (more sophisticated motion planning needed here)
                    rom_target_pd = pd_angle # Default to current
                    rom_target_ie = ie_angle # Default to current

                    if self._rom_state == "to_max_pd":
                        rom_target_pd = self.rom_max_pd
                        rom_target_ie = 0 # Keep IE centered during PD move (example)
                        if abs(pd_angle - self.rom_max_pd) < delta_limit:
                             self._rom_state = "to_min_pd" # Reached max, go to min
                             # Optional: Add hold time here
                    elif self._rom_state == "to_min_pd":
                         rom_target_pd = self.rom_min_pd
                         rom_target_ie = 0
                         if abs(pd_angle - self.rom_min_pd) < delta_limit:
                             self._rom_state = "to_max_ie" # Finished PD cycle, move to IE
                    elif self._rom_state == "to_max_ie":
                         rom_target_pd = 0 # Keep PD centered
                         rom_target_ie = self.rom_max_ie
                         if abs(ie_angle - self.rom_max_ie) < delta_limit:
                             self._rom_state = "to_min_ie"
                    elif self._rom_state == "to_min_ie":
                         rom_target_pd = 0
                         rom_target_ie = self.rom_min_ie
                         if abs(ie_angle - self.rom_min_ie) < delta_limit:
                            # Finished IE cycle, check reps
                            if self._rom_current_rep < self.rom_reps:
                                self._rom_current_rep += 1
                                self._rom_state = "to_max_pd" # Start next rep
                                self.log_message.emit(f"Rep {self._rom_current_rep}/{self.rom_reps}")
                            else:
                                self.log_message.emit("Passive ROM exercise finished.")
                                self._rom_state = "finished"
                                self.pause_control() # Auto-pause when done

                    # Set PID setpoints for ROM target
                    self.pd_pid.setpoint = rom_target_pd
                    self.ie_pid.setpoint = rom_target_ie

                    # Calculate PID output (speed could be limited differently here)
                    pd_output = self.pd_pid(pd_angle)
                    ie_output = self.ie_pid(ie_angle)
                    # Optional: Limit rate of change or use velocity control for smoother ROM


                elif mode == config.MODE_IDLE or mode == config.MODE_CALIBRATING:
                    # Keep motors stopped in Idle or Calibrating mode
                    pd_output = 0.0
                    ie_output = 0.0
                    # Reset PIDs?
                    self.pd_pid.reset()
                    self.ie_pid.reset()

                # --- Apply Output to Hardware ---
                # Final safety check before sending commands
                if not self._paused and not self._emergency_stopped:
                     # Clamp final output just in case PID overshoots limits
                    pd_output = max(-config.MAX_PWM_OUTPUT, min(config.MAX_PWM_OUTPUT, pd_output))
                    ie_output = max(-config.MAX_PWM_OUTPUT, min(config.MAX_PWM_OUTPUT, ie_output))

                    hw.set_motor_pwm(config.AXIS_PD, pd_output)
                    hw.set_motor_pwm(config.AXIS_IE, ie_output)

            except Exception as e:
                self.control_error.emit(f"Control loop error: {e}")
                # Consider putting system into a safe state (E-Stop?) on error
                self.emergency_stop() # Go to E-Stop on unexpected errors


            finally:
                 # Release mutex before sleeping
                 self._mutex.unlock()


            time.sleep(config.CONTROL_LOOP_INTERVAL) # Control loop frequency

        # --- Cleanup on thread exit ---
        print("Stopping motors before exiting ControlThread.")
        hw.set_motor_pwm(config.AXIS_PD, 0)
        hw.set_motor_pwm(config.AXIS_IE, 0)
        print("ControlThread finished.")

    def stop(self):
        self._mutex.lock()
        self._running = False
        self._paused = True # Ensure paused state
        self._wait_condition.wakeAll() # Wake up if waiting
        self._mutex.unlock()
        print("Stopping ControlThread...")