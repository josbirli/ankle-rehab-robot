# config.py
# constants and configuration settings.

# --- ROM Exercise Defaults ---
DEFAULT_ROM_MIN_PD = -10.0
DEFAULT_ROM_MAX_PD = 20.0
DEFAULT_ROM_MIN_IE = -5.0
DEFAULT_ROM_MAX_IE = 5.0
DEFAULT_ROM_SPEED_DPS = 5.0 # Degrees per second
DEFAULT_ROM_REPS = 3

MAX_SPEED_DPS = 30.0 # Max speed for exercises

MAX_PD_ANGLE = 40.0
MIN_PD_ANGLE = -20.0
MAX_IE_ANGLE = 25.0
MIN_IE_ANGLE = 15.0
MAX_SPEED_DPS = 30.0

MAX_PWM_OUTPUT = 100


##### PID Controller Gains #####
## !!! EDIT THIS LATER !!! ##

PD_PID_KP = 0.8
PD_PID_KI = 0.1
PD_PID_KD = 0.05

IE_PID_KP = 0.8
IE_PID_KI = 0.1 
IE_PID_KD = 0.05



##### Hardware Pins // These pins will be used with "RPi.GPIO" later. #####
## !!! EDIT THIS LATER !!! ##

MOTOR_PD_PWM_PIN = 12
MOTOR_PD_DIR1_PIN = 5
MOTOR_PD_DIR2_PIN = 6

MOTOR_IE_PWM_PIN = 13
MOTOR_IE_DIR1_PIN = 19
MOTOR_IE_DIR2_PIN = 26


##### SENSOR SETUP // MPU6050 ACCELEROMETER, (potentially an encoder) #####
## !!! EDIT THIS LATER !!! ###
MPU_SPI_PORT = 0
MPU_SPI_DEVICE = 0
MPU_PD_CHANNEL = 0
MPU_IE_CHANNEL = 1



##### THREAD TIMING #####
## !!! EDIT THIS LATER !!! ##

SENSOR_READ_INTERVAL = 0.02
CONTROL_LOOP_INTERVAL = 0.02


##### PLOTTING #####

PLOT_DATA_BUFFER_SIZE = 500


##### AXIS IDENTIFIERS #####

AXIS_PD = "Plantar - Dorsiflexion"
AXIS_IE = "Inversion - Eversion"


##### MODES #####

MODE_MANUAL = "Manual Control"
MODE_PASSIVE_ROM = "Passive ROM"
MODE_IDLE = "Idle"
MODE_ESTOP = "EMERGENCY STOPPED"
MODE_CALIBRATING = "Calibrating"
