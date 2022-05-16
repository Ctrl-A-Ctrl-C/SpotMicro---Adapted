#!/home/pi/spotmicroai/venv/bin/python3 -u

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pick import pick
import time
import os
import sys
import RPi.GPIO as GPIO

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config

log = Logger().setup_logger('CALIBRATE SERVOS')

log.info('Calibrate rest position...')

pca = None

gpio_port = Config().get(Config.ABORT_CONTROLLER_GPIO_PORT)

GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_port, GPIO.OUT)
GPIO.output(gpio_port, False)

i2c = busio.I2C(SCL, SDA)
options = {
    0: 'rear_shoulder_left   - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)) + ']',
    1: 'rear_leg_left        - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)) + ']',
    2: 'rear_feet_left       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)) + ']',
    3: 'rear_shoulder_right  - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)) + ']',
    4: 'rear_leg_right       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)) + ']',
    5: 'rear_feet_right      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)) + ']',
    6: 'front_shoulder_left  - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)) + ']',
    7: 'front_leg_left       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)) + ']',
    8: 'front_feet_left      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)) + ']',
    9: 'front_shoulder_right - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)) + ']',
    10: 'front_leg_right      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)) + ']',
    11: 'front_feet_right     - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)) + ']',
    12: 'arm_rotation         - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)) + ']',
    13: 'arm_lift             - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)) + ']',
    14: 'arm_range            - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)) + ']',
    15: 'arm_cam_tilt         - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL)) + '] - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)) + ']'}

title = 'The folder integration_tests for more tests' + os.linesep + \
        '1. Use "i2cdetect -y 1" to identify your i2c address' + os.linesep + \
        '2. Write your pca9685 i2c address(es) and settings in your configuration file ~/spotmicroai.json' + os.linesep + \
        '3. if no angle is specified 90 will be the default position, for example if you just press Enter' + os.linesep + \
        '' + os.linesep + \
        'Write "menu" or "m" and press Enter to return to the list of servos' + os.linesep + \
        'Write "exit" or "e" and press Enter to exit' + os.linesep + \
        '' + os.linesep + \
        'Please choose the servo to calibrate its rest position: '

#screen_options = list(options.values())

#selected_option, selected_index = pick(screen_options, title)
"""
# shoulders
shouders_idx = [0,3,6,9]
user_input = 90
for i,val in enumerate(shouders_idx):
    selected_option = options[val]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(user_input)
    time.sleep(0.1)
    pca.deinit()

# legs left
legs_left_idx = [1,7]
user_input = 120
for i,val in enumerate(legs_left_idx):
    selected_option = options[val]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(user_input)
    time.sleep(0.1)
    pca.deinit()

# legs right
legs_right_idx = [4,10]
user_input = 60
for i,val in enumerate(legs_right_idx):
    selected_option = options[val]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(user_input)
    time.sleep(0.1)
    pca.deinit()

# feet right
feet_right_idx = [5,11]
user_input = 0
for i,val in enumerate(feet_right_idx):
    selected_option = options[val]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(user_input)
    time.sleep(0.1)
    pca.deinit()

# feet left
feet_left_idx = [2,8]
user_input = 180
for i,val in enumerate(feet_left_idx):
    selected_option = options[val]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(user_input)
    time.sleep(0.1)
    pca.deinit()
"""

def servo_controller(motor, angle):
    selected_option = options[motor]
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
    pca.frequency = PCA9685_FREQUENCY

    active_servo = servo.Servo(pca.channels[CHANNEL])
    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

    active_servo.angle = int(angle)
    time.sleep(0.1)
    pca.deinit()

shouders_idx = [0,3,6,9]
angle = 90
for i,motor in enumerate(shouders_idx): 
    servo_controller(motor, angle)

legs_left_idx = [1,7]
angle = 120
for i,motor in enumerate(legs_left_idx):
    servo_controller(motor, angle)

legs_right_idx = [4,10]
angle = 60
for i,motor in enumerate(legs_right_idx):
    servo_controller(motor, angle)

feet_right_idx = [5,11]
angle = 0
for i,motor in enumerate(feet_right_idx):
    servo_controller(motor, angle)

feet_left_idx = [2,8]
angle = 180
for i,motor in enumerate(feet_left_idx):
    servo_controller(motor, angle)

print('Finished auto-calibration')






