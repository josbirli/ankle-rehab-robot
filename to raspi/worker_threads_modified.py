# worker_threads_modified.py
import time
import threading
from queue import Queue
from simple_pid import PID

import hardware_interface as hw
import config # Use the mock config

class SensorThread(threading.Thread):
    def __init__(self, angle_callback, error_callback):
        super().__init__(daemon=True)
        self._running = False
        self._lock = threading.Lock()
        self.angle_callback = angle_callback
        self.error_callback = error_callback
        self.name = "SensorThread"

    def run(self):
        with self._lock:
            self._running = True

        print(f"{self.name}: Started.")
        while True:
            with self._lock:
                if not self._running:
                    break # Exit loop

            # Code executed without holding lock for the sleep duration
            try:
                pd_angle = hw.read_sensor(config.AXIS_PD)
                ie_angle = hw.read_sensor(config.AXIS_IE)

                if pd_angle is not None and ie_angle is not None:
                    if self.angle_callback:
                        self.angle_callback(pd_angle, ie_angle)
                else:
                    if pd_angle is None and self.error_callback:
                        self.error_callback("Failed to read PD sensor.")
                    if ie_angle is None and self.error_callback:
                        self.error_callback("Failed to read IE sensor.")

            except Exception as e:
                if self.error_callback:
                    self.error_callback(f"Sensor thread error: {e}")

            time.sleep(config.SENSOR_READ_INTERVAL)
        print(f"{self.name}: Finished.")

    def stop(self):
        print(f"Stopping {self.name}...")
        with self._lock:
            self._running = False


class ControlThread(threading.Thread):
    def __init__(self, output_queue: Queue):
        super().__init__(daemon=True)
        self.output_queue = output_queue # For logs, status, errors
        self._running = False
        self._paused = True
        self._emergency_stopped = False
        self._lock = threading.Lock()
        self._pause_condition = threading.Condition(self._lock)
        self.name = "ControlThread"

        self.current_mode = config.MODE_IDLE
        self.target_pd_angle = 0.0
        self.target_ie_angle = 0.0
        self.current_pd_angle = 0.0
        self.current_ie_angle = 0.0
        self.manual_speed_factor = 0.5

        # PID Controllers
        self.pd_pid = PID(config.PD_PID_KP, config.PD_PID_KI, config.PD_PID_KD, setpoint=0)
        self.ie_pid = PID(config.IE_PID_KP, config.IE_PID_KI, config.IE_PID_KD, setpoint=0)
        self.pd_pid.output_limits = (-config.MAX_PWM_OUTPUT, config.MAX_PWM_OUTPUT)
        self.ie_pid.output_limits = (-config.MAX_PWM_OUTPUT, config.MAX_PWM_OUTPUT)
        self.pd_pid.sample_time = config.CONTROL_LOOP_INTERVAL
        self.ie_pid.sample_time = config.CONTROL_LOOP_INTERVAL

        self.rom_min_pd = config.DEFAULT_ROM_MIN_PD
        self.rom_max_pd = config.DEFAULT_ROM_MAX_PD
        self.rom_min_ie = config.DEFAULT_ROM_MIN_IE
        self.rom_max_ie = config.DEFAULT_ROM_MAX_IE
        self.rom_speed_dps = config.DEFAULT_ROM_SPEED_DPS
        self.rom_reps = config.DEFAULT_ROM_REPS
        self._rom_current_rep = 0
        self._rom_state = "idle"

    def _send_output(self, type, content_key, content_value):
        """Helper to put messages on the output queue."""
        self.output_queue.put({"type": type, content_key: content_value, "timestamp": time.time()})

    def _log(self, message):
        self._send_output("log", "message", message)

    def _status_update(self, status):
        self._send_output("status", "status", status)

    def _control_error(self, error_message):
        self._send_output("error", "message", f"Control Error: {error_message}")

    def update_current_angles(self, pd_angle, ie_angle):
        with self._lock:
            self.current_pd_angle = pd_angle
            self.current_ie_angle = ie_angle

    def set_target_angles(self, pd_target, ie_target):
        with self._lock:
            self.target_pd_angle = pd_target
            self.target_ie_angle = ie_target
            self.pd_pid.setpoint = self.target_pd_angle
            self.ie_pid.setpoint = self.target_ie_angle
            # self._log(f"Targets: PD={pd_target:.1f}, IE={ie_target:.1f}")

    def set_manual_speed(self, speed_percentage):
        with self._lock:
            speed_percentage = max(0.0, min(100.0, speed_percentage))
            self.manual_speed_factor = speed_percentage / 100.0
            self._log(f"Manual speed factor: {self.manual_speed_factor:.2f}")

    def set_mode(self, mode):
        with self._lock:
            if mode != self.current_mode:
                self._log(f"Changing mode to: {mode}")
                self.current_mode = mode
                self._paused = True # Pause when changing mode
                self._reset_exercise_state()
                self.pd_pid.reset()
                self.ie_pid.reset()
                hw.set_motor_pwm(config.AXIS_PD, 0)
                hw.set_motor_pwm(config.AXIS_IE, 0)
                self._status_update(f"Mode: {self.current_mode} (Paused)")
                if self._paused: # Ensure pause condition is notified if already waiting
                    self._pause_condition.notify_all()


    def set_rom_params(self, min_pd, max_pd, min_ie, max_ie, speed, reps):
        with self._lock:
            self.rom_min_pd = min_pd
            self.rom_max_pd = max_pd
            self.rom_min_ie = min_ie
            self.rom_max_ie = max_ie
            self.rom_speed_dps = max(1.0, speed)
            self.rom_reps = reps
            self._log(f"Set ROM Params: PD[{min_pd:.1f},{max_pd:.1f}], IE[{min_ie:.1f},{max_ie:.1f}], Speed:{speed:.1f}, Reps:{reps}")

    def _reset_exercise_state(self):
        # Assumed to be called with lock held
        self._rom_current_rep = 0
        self._rom_state = "idle"
        self.target_pd_angle = 0.0 # Reset to current or zero?
        self.target_ie_angle = 0.0
        self.pd_pid.setpoint = 0.0
        self.ie_pid.setpoint = 0.0

    def start_control(self):
        with self._lock:
            if not self._emergency_stopped:
                if self._paused: # Only change if actually paused
                    self._paused = False
                    self._log("Control started.")
                    self._status_update(f"Running: {self.current_mode}")
                    self._pause_condition.notify_all() # Wake up if paused
            else:
                self._log("Cannot start, E-Stop is active.")

    def pause_control(self):
        with self._lock:
            if not self._paused: # Only change if not already paused
                self._paused = True
                hw.set_motor_pwm(config.AXIS_PD, 0)
                hw.set_motor_pwm(config.AXIS_IE, 0)
                self._log("Control paused.")
                self._status_update(f"Paused: {self.current_mode}")

    def emergency_stop(self):
        with self._lock:
            self._emergency_stopped = True
            self._paused = True
            hw.set_motor_pwm(config.AXIS_PD, 0)
            hw.set_motor_pwm(config.AXIS_IE, 0)
            self._status_update(config.MODE_ESTOP)
            self._log("--- EMERGENCY STOP ACTIVATED ---")
            if self._paused: # Ensure pause condition is notified
                 self._pause_condition.notify_all()


    def reset_emergency_stop(self):
        with self._lock:
            if self._emergency_stopped:
                self._emergency_stopped = False
                self._paused = True # Remain paused
                self.current_mode = config.MODE_IDLE
                self._reset_exercise_state()
                self._log("Emergency stop reset. System Idle and Paused.")
                self._status_update(config.MODE_IDLE)


    def run(self):
        print(f"{self.name}: Started.")
        with self._lock:
            self._running = True
            self._paused = True
            self._emergency_stopped = False
            self.current_mode = config.MODE_IDLE
            self._status_update(config.MODE_IDLE)

        while True:
            # Lock is acquired for the whole loop iteration to access shared state
            with self._lock:
                if not self._running:
                    break # Exit thread loop

                if self._emergency_stopped:
                    hw.set_motor_pwm(config.AXIS_PD, 0)
                    hw.set_motor_pwm(config.AXIS_IE, 0)
                    # Unlock before sleep, re-lock after. Condition handles this.
                    self._pause_condition.wait(0.1) # Wait with timeout or until notified
                    continue

                if self._paused:
                    self._pause_condition.wait() # Wait until woken
                    if not self._running or self._emergency_stopped: # Re-check flags
                        continue

                # --- Main Control Logic (lock is held) ---
                mode = self.current_mode
                pd_angle = self.current_pd_angle
                ie_angle = self.current_ie_angle
                target_pd = self.target_pd_angle
                target_ie = self.target_ie_angle
                speed_factor = self.manual_speed_factor

                safe_pd_target = max(config.MIN_PD_ANGLE, min(config.MAX_PD_ANGLE, target_pd))
                safe_ie_target = max(config.MIN_IE_ANGLE, min(config.MAX_IE_ANGLE, target_ie))

                if safe_pd_target != target_pd:
                    self._log(f"Warning: PD Target ({target_pd:.1f}) clamped to {safe_pd_target:.1f}")
                    self.target_pd_angle = safe_pd_target
                    self.pd_pid.setpoint = safe_pd_target
                if safe_ie_target != target_ie:
                    self._log(f"Warning: IE Target ({target_ie:.1f}) clamped to {safe_ie_target:.1f}")
                    self.target_ie_angle = safe_ie_target
                    self.ie_pid.setpoint = safe_ie_target

                pd_output = 0.0
                ie_output = 0.0

                try:
                    if mode == config.MODE_MANUAL:
                        pd_output = self.pd_pid(pd_angle)
                        ie_output = self.ie_pid(ie_angle)
                        pd_output *= speed_factor
                        ie_output *= speed_factor

                    elif mode == config.MODE_PASSIVE_ROM:
                        delta_limit = 1.5 # Degrees close enough to target for ROM

                        if self._rom_state == "idle":
                            self._log("Starting Passive ROM exercise.")
                            self._rom_current_rep = 1
                            self._rom_state = "to_max_pd"
                            self._log(f"Rep {self._rom_current_rep}/{self.rom_reps}")

                        rom_target_pd, rom_target_ie = pd_angle, ie_angle # Default to current

                        if self._rom_state == "to_max_pd":
                            rom_target_pd = self.rom_max_pd; rom_target_ie = 0
                            if abs(pd_angle - self.rom_max_pd) < delta_limit: self._rom_state = "to_min_pd"
                        elif self._rom_state == "to_min_pd":
                            rom_target_pd = self.rom_min_pd; rom_target_ie = 0
                            if abs(pd_angle - self.rom_min_pd) < delta_limit: self._rom_state = "to_max_ie"
                        elif self._rom_state == "to_max_ie":
                            rom_target_pd = 0; rom_target_ie = self.rom_max_ie
                            if abs(ie_angle - self.rom_max_ie) < delta_limit: self._rom_state = "to_min_ie"
                        elif self._rom_state == "to_min_ie":
                            rom_target_pd = 0; rom_target_ie = self.rom_min_ie
                            if abs(ie_angle - self.rom_min_ie) < delta_limit:
                                if self._rom_current_rep < self.rom_reps:
                                    self._rom_current_rep += 1
                                    self._rom_state = "to_max_pd"
                                    self._log(f"Rep {self._rom_current_rep}/{self.rom_reps}")
                                else:
                                    self._log("Passive ROM exercise finished.")
                                    self._rom_state = "finished"
                                    self.pause_control() # Auto-pause

                        self.pd_pid.setpoint = rom_target_pd
                        self.ie_pid.setpoint = rom_target_ie
                        pd_output = self.pd_pid(pd_angle)
                        ie_output = self.ie_pid(ie_angle)
                        # Consider ROM specific speed scaling here if needed

                    elif mode == config.MODE_IDLE or mode == config.MODE_CALIBRATING:
                        pd_output = 0.0; ie_output = 0.0
                        self.pd_pid.reset(); self.ie_pid.reset()

                    # Apply Output to Hardware (only if not paused and not estopped)
                    # These checks are implicitly handled by the pause/estop logic at loop start
                    pd_output_clamped = max(-config.MAX_PWM_OUTPUT, min(config.MAX_PWM_OUTPUT, pd_output))
                    ie_output_clamped = max(-config.MAX_PWM_OUTPUT, min(config.MAX_PWM_OUTPUT, ie_output))
                    hw.set_motor_pwm(config.AXIS_PD, pd_output_clamped)
                    hw.set_motor_pwm(config.AXIS_IE, ie_output_clamped)

                except Exception as e:
                    self._control_error(f"Control loop error: {e}")
                    self.emergency_stop() # Critical error in control logic

            # Unlock before sleep
            time.sleep(config.CONTROL_LOOP_INTERVAL)

        # --- Cleanup on thread exit ---
        self._log(f"Stopping motors before exiting {self.name}.")
        hw.set_motor_pwm(config.AXIS_PD, 0)
        hw.set_motor_pwm(config.AXIS_IE, 0)
        print(f"{self.name}: Finished.")

    def stop(self):
        print(f"Stopping {self.name}...")
        with self._lock:
            self._running = False
            if self._paused or self._emergency_stopped: # if waiting on condition
                self._pause_condition.notify_all()