# hardware_interface.py
# <<< Replace this with your ACTUAL hardware control code >>>
# This is a DUMMY interface for testing the GUI structure.

import time
import random
import config

# --- Dummy State ---
# Simulate sensor readings and motor outputs
dummy_current_pd_angle = 0.0
dummy_current_ie_angle = 0.0
dummy_pd_pwm = 0
dummy_ie_pwm = 0
is_gpio_setup = False

# --- Motor Identifiers (Match config.py) ---
MOTOR_PD = config.AXIS_PD
MOTOR_IE = config.AXIS_IE

def setup_gpio():
    """Initialize GPIO pins for motors and sensors."""
    global is_gpio_setup
    print("[Hardware] Initializing GPIO...")
    # --- Your RPi.GPIO or pigpio setup code here ---
    # Example using RPi.GPIO (needs sudo)
    # import RPi.GPIO as GPIO
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(config.MOTOR_PD_DIR1_PIN, GPIO.OUT)
    # GPIO.setup(config.MOTOR_PD_DIR2_PIN, GPIO.OUT)
    # GPIO.setup(config.MOTOR_PD_PWM_PIN, GPIO.OUT)
    # pd_pwm = GPIO.PWM(config.MOTOR_PD_PWM_PIN, 1000) # 1kHz frequency
    # pd_pwm.start(0)
    # ... setup IE motor pins ...
    # ... setup sensor pins/interfaces (GPIO for encoders, SPI/I2C for ADC)...
    # --------------------------------------------------
    print("[Hardware] GPIO Initialized (Dummy).")
    is_gpio_setup = True
    # Ensure motors start stopped
    set_motor_pwm(MOTOR_PD, 0)
    set_motor_pwm(MOTOR_IE, 0)


def set_motor_pwm(axis, pwm_value):
    """
    Set the PWM duty cycle and direction for a motor.

    Args:
        axis (str): MOTOR_PD or MOTOR_IE.
        pwm_value (float): -100 to +100. Negative for one direction, positive for the other.
                           0 means stop.
    """
    global dummy_pd_pwm, dummy_ie_pwm
    if not is_gpio_setup:
        print("[Hardware] Error: GPIO not setup.")
        return

    # Clamp value to range
    pwm_value = max(-config.MAX_PWM_OUTPUT, min(config.MAX_PWM_OUTPUT, pwm_value))

    # --- Your actual motor driver control code here ---
    # Determine direction based on sign of pwm_value
    # Set direction pins accordingly
    # Set PWM duty cycle using abs(pwm_value)
    # Example using RPi.GPIO:
    # duty_cycle = abs(pwm_value)
    # if axis == MOTOR_PD:
    #     if pwm_value > 0: # Plantarflexion (example direction)
    #         GPIO.output(config.MOTOR_PD_DIR1_PIN, GPIO.HIGH)
    #         GPIO.output(config.MOTOR_PD_DIR2_PIN, GPIO.LOW)
    #     elif pwm_value < 0: # Dorsiflexion
    #         GPIO.output(config.MOTOR_PD_DIR1_PIN, GPIO.LOW)
    #         GPIO.output(config.MOTOR_PD_DIR2_PIN, GPIO.HIGH)
    #     else: # Stop
    #         GPIO.output(config.MOTOR_PD_DIR1_PIN, GPIO.LOW)
    #         GPIO.output(config.MOTOR_PD_DIR2_PIN, GPIO.LOW)
    #     # pd_pwm.ChangeDutyCycle(duty_cycle) # Need pd_pwm object from setup
    #     dummy_pd_pwm = pwm_value
    # elif axis == MOTOR_IE:
    #     # ... similar logic for IE motor ...
    #     dummy_ie_pwm = pwm_value
    # --------------------------------------------------

    # Dummy simulation update
    if axis == MOTOR_PD:
        dummy_pd_pwm = pwm_value
        print(f"[Hardware] Set PD Motor PWM: {pwm_value:.1f}")
    elif axis == MOTOR_IE:
        dummy_ie_pwm = pwm_value
        print(f"[Hardware] Set IE Motor PWM: {pwm_value:.1f}")


def read_sensor(axis):
    """
    Read the current angle from the specified sensor.

    Args:
        axis (str): AXIS_PD or AXIS_IE.

    Returns:
        float: Current angle in degrees, or None if read fails.
    """
    global dummy_current_pd_angle, dummy_current_ie_angle
    if not is_gpio_setup:
        print("[Hardware] Error: GPIO not setup for sensor reading.")
        return None

    # --- Your actual sensor reading code here ---
    # Read encoder counts and convert to degrees, or
    # Read ADC value for potentiometer and map to degrees.
    # Example for dummy simulation:
    # Simulate slight movement based on motor pwm and noise
    noise = random.uniform(-0.2, 0.2)
    if axis == config.AXIS_PD:
        # Simulate response: angle changes slowly based on PWM command
        dummy_current_pd_angle += dummy_pd_pwm * 0.005 + noise
        # Clamp to limits for simulation realism
        dummy_current_pd_angle = max(config.MIN_PD_ANGLE - 5, min(config.MAX_PD_ANGLE + 5, dummy_current_pd_angle))
        # print(f"[Hardware] Read PD Angle: {dummy_current_pd_angle:.2f}")
        return dummy_current_pd_angle
    elif axis == config.AXIS_IE:
        dummy_current_ie_angle += dummy_ie_pwm * 0.005 + noise
        dummy_current_ie_angle = max(config.MIN_IE_ANGLE - 5, min(config.MAX_IE_ANGLE + 5, dummy_current_ie_angle))
        # print(f"[Hardware] Read IE Angle: {dummy_current_ie_angle:.2f}")
        return dummy_current_ie_angle
    # --------------------------------------------------

    return None # Should not happen in dummy, but good practice

def cleanup_gpio():
    """Release GPIO resources."""
    global is_gpio_setup
    print("[Hardware] Cleaning up GPIO...")
    # --- Your RPi.GPIO cleanup code here ---
    # Example:
    # import RPi.GPIO as GPIO
    # if is_gpio_setup:
    #    set_motor_pwm(MOTOR_PD, 0) # Stop motors before cleanup
    #    set_motor_pwm(MOTOR_IE, 0)
    #    # pd_pwm.stop() # Need pd_pwm object
    #    # ie_pwm.stop()
    #    GPIO.cleanup()
    # ------------------------------------
    is_gpio_setup = False
    print("[Hardware] GPIO Cleaned up (Dummy).")