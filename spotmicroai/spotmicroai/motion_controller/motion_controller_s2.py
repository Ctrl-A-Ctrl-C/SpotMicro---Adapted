import signal
import sys

import queue
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import time
import numpy as np

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config
import spotmicroai.utilities.queues as queues
from spotmicroai.utilities.general import General

from spotmicroai.motion_controller.Spotmicro_Inverse_Kinematics_and_Position_Library_v01 import Spot
Spot = Spot()
from spotmicroai.motion_controller.Spotmicro_Gravity_Center_Library_v01 import SpotCG
SpotCG = SpotCG()
from math import pi, sin, cos, atan2, sqrt

log = Logger().setup_logger('Motion controller')


class MotionController:
    boards = 1

    is_activated = False

    i2c = None
    pca9685_1 = None
    pca9685_2 = None

    pca9685_1_address = None
    pca9685_1_reference_clock_speed = None
    pca9685_1_frequency = None
    pca9685_2_address = None
    pca9685_2_reference_clock_speed = None
    pca9685_2_frequency = None

    servo_rear_shoulder_left = None
    servo_rear_shoulder_left_pca9685 = None
    servo_rear_shoulder_left_channel = None
    servo_rear_shoulder_left_min_pulse = None
    servo_rear_shoulder_left_max_pulse = None
    servo_rear_shoulder_left_rest_angle = None

    servo_rear_leg_left = None
    servo_rear_leg_left_pca9685 = None
    servo_rear_leg_left_channel = None
    servo_rear_leg_left_min_pulse = None
    servo_rear_leg_left_max_pulse = None
    servo_rear_leg_left_rest_angle = None

    servo_rear_feet_left = None
    servo_rear_feet_left_pca9685 = None
    servo_rear_feet_left_channel = None
    servo_rear_feet_left_min_pulse = None
    servo_rear_feet_left_max_pulse = None
    servo_rear_feet_left_rest_angle = None

    servo_rear_shoulder_right = None
    servo_rear_shoulder_right_pca9685 = None
    servo_rear_shoulder_right_channel = None
    servo_rear_shoulder_right_min_pulse = None
    servo_rear_shoulder_right_max_pulse = None
    servo_rear_shoulder_right_rest_angle = None

    servo_rear_leg_right = None
    servo_rear_leg_right_pca9685 = None
    servo_rear_leg_right_channel = None
    servo_rear_leg_right_min_pulse = None
    servo_rear_leg_right_max_pulse = None
    servo_rear_leg_right_rest_angle = None

    servo_rear_feet_right = None
    servo_rear_feet_right_pca9685 = None
    servo_rear_feet_right_channel = None
    servo_rear_feet_right_min_pulse = None
    servo_rear_feet_right_max_pulse = None
    servo_rear_feet_right_rest_angle = None

    servo_front_shoulder_left = None
    servo_front_shoulder_left_pca9685 = None
    servo_front_shoulder_left_channel = None
    servo_front_shoulder_left_min_pulse = None
    servo_front_shoulder_left_max_pulse = None
    servo_front_shoulder_left_rest_angle = None

    servo_front_leg_left = None
    servo_front_leg_left_pca9685 = None
    servo_front_leg_left_channel = None
    servo_front_leg_left_min_pulse = None
    servo_front_leg_left_max_pulse = None
    servo_front_leg_left_rest_angle = None

    servo_front_feet_left = None
    servo_front_feet_left_pca9685 = None
    servo_front_feet_left_channel = None
    servo_front_feet_left_min_pulse = None
    servo_front_feet_left_max_pulse = None
    servo_front_feet_left_rest_angle = None

    servo_front_shoulder_right = None
    servo_front_shoulder_right_pca9685 = None
    servo_front_shoulder_right_channel = None
    servo_front_shoulder_right_min_pulse = None
    servo_front_shoulder_right_max_pulse = None
    servo_front_shoulder_right_rest_angle = None

    servo_front_leg_right = None
    servo_front_leg_right_pca9685 = None
    servo_front_leg_right_channel = None
    servo_front_leg_right_min_pulse = None
    servo_front_leg_right_max_pulse = None
    servo_front_leg_right_rest_angle = None

    servo_front_feet_right = None
    servo_front_feet_right_pca9685 = None
    servo_front_feet_right_channel = None
    servo_front_feet_right_min_pulse = None
    servo_front_feet_right_max_pulse = None
    servo_front_feet_right_rest_angle = None

    servo_arm_rotation = None
    servo_arm_rotation_pca9685 = None
    servo_arm_rotation_channel = None
    servo_arm_rotation_min_pulse = None
    servo_arm_rotation_max_pulse = None
    servo_arm_rotation_rest_angle = None

    servo_arm_lift = None
    servo_arm_lift_pca9685 = None
    servo_arm_lift_channel = None
    servo_arm_lift_min_pulse = None
    servo_arm_lift_max_pulse = None
    servo_arm_lift_rest_angle = None

    servo_arm_range = None
    servo_arm_range_pca9685 = None
    servo_arm_range_channel = None
    servo_arm_range_min_pulse = None
    servo_arm_range_max_pulse = None
    servo_arm_range_rest_angle = None

    servo_arm_cam_tilt = None
    servo_arm_cam_tilt_pca9685 = None
    servo_arm_cam_tilt_channel = None
    servo_arm_cam_tilt_min_pulse = None
    servo_arm_cam_tilt_max_pulse = None
    servo_arm_cam_tilt_rest_angle = None

    def __init__(self, communication_queues):

        try:

            log.debug('Starting controller...')

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            self.i2c = busio.I2C(SCL, SDA)
            self.load_pca9685_boards_configuration()
            self.load_servos_configuration()

            self._abort_queue = communication_queues[queues.ABORT_CONTROLLER]
            self._motion_queue = communication_queues[queues.MOTION_CONTROLLER]
            self._lcd_screen_queue = communication_queues[queues.LCD_SCREEN_CONTROLLER]

            if self.pca9685_2_address:
                self._lcd_screen_queue.put('motion_controller_1 OK')
                self._lcd_screen_queue.put('motion_controller_2 OK')
            else:
                self._lcd_screen_queue.put('motion_controller_1 OK')
                self._lcd_screen_queue.put('motion_controller_2 NOK')

            self._previous_event = {}

        except Exception as e:
            log.error('Motion controller initialization problem', e)
            self._lcd_screen_queue.put('motion_controller_1 NOK')
            self._lcd_screen_queue.put('motion_controller_2 NOK')
            try:
                self.pca9685_1.deinit()
            finally:
                try:
                    if self.boards == 2:
                        self.pca9685_2.deinit()
                finally:
                    sys.exit(1)

    def exit_gracefully(self, signum, frame):
        try:
            self.pca9685_1.deinit()
        finally:
            try:
                if self.boards == 2:
                    self.pca9685_2.deinit()
            finally:
                log.info('Terminated')
                sys.exit(0)

    def do_process_events_from_queues(self):

        while True:

            try:

                event = self._motion_queue.get(block=True, timeout=60)

                # log.debug(event)

                if event['start']:
                    if self.is_activated:
                        self.rest_position()
                        time.sleep(0.5)
                        self.deactivate_pca9685_boards()
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                    else:
                        self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ACTIVATE)
                        self.activate_pca9685_boards()
                        self.activate_servos()
                        self.rest_position()

                if not self.is_activated:
                    log.info('Press START/OPTIONS to enable the servos')
                    continue

                # btn A
                if event['a']:
                    self.auto_walking()

                # dpad
                if event['hat0y']:
                    self.body_move_body_up_and_down(event['hat0y'])

                # dpad
                if event['hat0x']:
                    self.body_move_body_left_right(event['hat0x'])

                # right joystick
                #if event['ry']:
                 #   self.body_move_body_up_and_down_analog(event['ry'])

                # right joystick
                if event['rx']:
                    self.stop_turn(event['rx'])

                if event['hat0x'] and event['tl2']:
                    # 2 buttons example
                    pass

                # btn Y
                if event['y']:
                    self.standing()

                # btn B
                if event['b']:
                    self.sit()

                # btn X
                if event['x']:
                    self.pee()

                # trigger left - hold to control
                if event['tl']:
                    self.arm_set_rotation(event['lx'])

                if event['tl']:
                    self.arm_set_lift(event['ly'])

                # trigger left - hold to control
                if event['tr']:
                    self.arm_set_range(event['ly'])

                if event['tr']:
                    self.arm_set_cam_tilt(event['ry'])

                self.move()

            except queue.Empty as e:
                log.info('Inactivity lasted 60 seconds, shutting down the servos, '
                         'press start to reactivate')
                if self.is_activated:
                    self.rest_position()
                    time.sleep(0.5)
                    self.deactivate_pca9685_boards()

            except Exception as e:
                log.error('Unknown problem while processing the queue of the motion controller')
                log.error(' - Most likely a servo is not able to get to the assigned position')
                log.error(e)

    def load_pca9685_boards_configuration(self):
        self.pca9685_1_address = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_ADDRESS), 0)
        self.pca9685_1_reference_clock_speed = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_REFERENCE_CLOCK_SPEED))
        self.pca9685_1_frequency = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_1_FREQUENCY))

        self.pca9685_2_address = False
        try:
            self.pca9685_2_address = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_ADDRESS), 0)

            if self.pca9685_2_address:
                self.pca9685_2_reference_clock_speed = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_REFERENCE_CLOCK_SPEED))
                self.pca9685_2_frequency = int(Config().get(Config.MOTION_CONTROLLER_BOARDS_PCA9685_2_FREQUENCY))

        except Exception as e:
            log.debug("Only 1 PCA9685 is present in the configuration")

    def activate_pca9685_boards(self):

        self.pca9685_1 = PCA9685(self.i2c, address=self.pca9685_1_address,
                                 reference_clock_speed=self.pca9685_1_reference_clock_speed)
        self.pca9685_1.frequency = self.pca9685_1_frequency

        if self.pca9685_2_address:
            self.pca9685_2 = PCA9685(self.i2c, address=self.pca9685_2_address,
                                     reference_clock_speed=self.pca9685_2_reference_clock_speed)
            self.pca9685_2.frequency = self.pca9685_2_frequency
            self.boards = 2

        self.is_activated = True
        log.debug(str(self.boards) + ' PCA9685 board(s) activated')

    def deactivate_pca9685_boards(self):

        try:
            if self.pca9685_1:
                self.pca9685_1.deinit()
        finally:
            try:
                if self.boards == 2 and self.pca9685_2:
                    self.pca9685_2.deinit()
            finally:
                # self._abort_queue.put(queues.ABORT_CONTROLLER_ACTION_ABORT)
                self.is_activated = False

        log.debug(str(self.boards) + ' PCA9685 board(s) deactivated')

    def load_servos_configuration(self):

        self.servo_rear_shoulder_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)
        self.servo_rear_shoulder_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL)
        self.servo_rear_shoulder_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MIN_PULSE)
        self.servo_rear_shoulder_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_MAX_PULSE)
        self.servo_rear_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)

        self.servo_rear_leg_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)
        self.servo_rear_leg_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL)
        self.servo_rear_leg_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MIN_PULSE)
        self.servo_rear_leg_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_MAX_PULSE)
        self.servo_rear_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)

        self.servo_rear_feet_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685)
        self.servo_rear_feet_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL)
        self.servo_rear_feet_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MIN_PULSE)
        self.servo_rear_feet_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_MAX_PULSE)
        self.servo_rear_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)

        self.servo_rear_shoulder_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)
        self.servo_rear_shoulder_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL)
        self.servo_rear_shoulder_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MIN_PULSE)
        self.servo_rear_shoulder_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_MAX_PULSE)
        self.servo_rear_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)

        self.servo_rear_leg_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)
        self.servo_rear_leg_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL)
        self.servo_rear_leg_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MIN_PULSE)
        self.servo_rear_leg_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_MAX_PULSE)
        self.servo_rear_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)

        self.servo_rear_feet_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685)
        self.servo_rear_feet_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL)
        self.servo_rear_feet_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MIN_PULSE)
        self.servo_rear_feet_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_MAX_PULSE)
        self.servo_rear_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)

        self.servo_front_shoulder_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)
        self.servo_front_shoulder_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL)
        self.servo_front_shoulder_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MIN_PULSE)
        self.servo_front_shoulder_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_MAX_PULSE)
        self.servo_front_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)

        self.servo_front_leg_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)
        self.servo_front_leg_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL)
        self.servo_front_leg_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MIN_PULSE)
        self.servo_front_leg_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_MAX_PULSE)
        self.servo_front_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)

        self.servo_front_feet_left_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685)
        self.servo_front_feet_left_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL)
        self.servo_front_feet_left_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MIN_PULSE)
        self.servo_front_feet_left_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_MAX_PULSE)
        self.servo_front_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)

        self.servo_front_shoulder_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)
        self.servo_front_shoulder_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL)
        self.servo_front_shoulder_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MIN_PULSE)
        self.servo_front_shoulder_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_MAX_PULSE)
        self.servo_front_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)

        self.servo_front_leg_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)
        self.servo_front_leg_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL)
        self.servo_front_leg_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MIN_PULSE)
        self.servo_front_leg_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_MAX_PULSE)
        self.servo_front_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)

        self.servo_front_feet_right_pca9685 = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685)
        self.servo_front_feet_right_channel = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL)
        self.servo_front_feet_right_min_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MIN_PULSE)
        self.servo_front_feet_right_max_pulse = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_MAX_PULSE)
        self.servo_front_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)

        if self.servo_arm_rotation_pca9685:
            self.servo_arm_rotation_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685)
            self.servo_arm_rotation_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL)
            self.servo_arm_rotation_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_MIN_PULSE)
            self.servo_arm_rotation_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_MAX_PULSE)
            self.servo_arm_rotation_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)

            self.servo_arm_lift_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685)
            self.servo_arm_lift_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL)
            self.servo_arm_lift_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_MIN_PULSE)
            self.servo_arm_lift_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_MAX_PULSE)
            self.servo_arm_lift_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)

            self.servo_arm_range_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685)
            self.servo_arm_range_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL)
            self.servo_arm_range_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_MIN_PULSE)
            self.servo_arm_range_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_MAX_PULSE)
            self.servo_arm_range_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)

            self.servo_arm_cam_tilt_pca9685 = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685)
            self.servo_arm_cam_tilt_channel = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL)
            self.servo_arm_cam_tilt_min_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MIN_PULSE)
            self.servo_arm_cam_tilt_max_pulse = Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MAX_PULSE)
            self.servo_arm_cam_tilt_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)

    def activate_servos(self):

        if self.servo_rear_shoulder_left_pca9685 == 1:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_left_channel])
        else:
            self.servo_rear_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_left_channel])
        self.servo_rear_shoulder_left.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_left_min_pulse, max_pulse=self.servo_rear_shoulder_left_max_pulse)

        if self.servo_rear_leg_left_pca9685 == 1:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_left_channel])
        else:
            self.servo_rear_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_left_channel])
        self.servo_rear_leg_left.set_pulse_width_range(min_pulse=self.servo_rear_leg_left_min_pulse, max_pulse=self.servo_rear_leg_left_max_pulse)

        if self.servo_rear_feet_left_pca9685 == 1:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_left_channel])
        else:
            self.servo_rear_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_left_channel])
        self.servo_rear_feet_left.set_pulse_width_range(min_pulse=self.servo_rear_feet_left_min_pulse, max_pulse=self.servo_rear_feet_left_max_pulse)

        if self.servo_rear_shoulder_right_pca9685 == 1:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_shoulder_right_channel])
        else:
            self.servo_rear_shoulder_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_shoulder_right_channel])
        self.servo_rear_shoulder_right.set_pulse_width_range(min_pulse=self.servo_rear_shoulder_right_min_pulse, max_pulse=self.servo_rear_shoulder_right_max_pulse)

        if self.servo_rear_leg_right_pca9685 == 1:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_leg_right_channel])
        else:
            self.servo_rear_leg_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_leg_right_channel])
        self.servo_rear_leg_right.set_pulse_width_range(min_pulse=self.servo_rear_leg_right_min_pulse, max_pulse=self.servo_rear_leg_right_max_pulse)

        if self.servo_rear_feet_right_pca9685 == 1:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_rear_feet_right_channel])
        else:
            self.servo_rear_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_rear_feet_right_channel])
        self.servo_rear_feet_right.set_pulse_width_range(min_pulse=self.servo_rear_feet_right_min_pulse, max_pulse=self.servo_rear_feet_right_max_pulse)

        if self.servo_front_shoulder_left_pca9685 == 1:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_left_channel])
        else:
            self.servo_front_shoulder_left = servo.Servo(self.pca9685_2.channels[self.servo_front_shoulder_left_channel])
        self.servo_front_shoulder_left.set_pulse_width_range(min_pulse=self.servo_front_shoulder_left_min_pulse, max_pulse=self.servo_front_shoulder_left_max_pulse)

        if self.servo_front_leg_left_pca9685 == 1:
            self.servo_front_leg_left = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_left_channel])
        else:
            self.servo_front_leg_left = servo.Servo(self.pca9685_2.channels[self.servo_front_leg_left_channel])
        self.servo_front_leg_left.set_pulse_width_range(min_pulse=self.servo_front_leg_left_min_pulse, max_pulse=self.servo_front_leg_left_max_pulse)

        if self.servo_front_feet_left_pca9685 == 1:
            self.servo_front_feet_left = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_left_channel])
        else:
            self.servo_front_feet_left = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_left_channel])
        self.servo_front_feet_left.set_pulse_width_range(min_pulse=self.servo_front_feet_left_min_pulse, max_pulse=self.servo_front_feet_left_max_pulse)

        if self.servo_front_shoulder_right_pca9685 == 1:
            self.servo_front_shoulder_right = servo.Servo(self.pca9685_1.channels[self.servo_front_shoulder_right_channel])
        else:
            self.servo_front_shoulder_right = servo.Servo(
                self.pca9685_2.channels[self.servo_front_shoulder_right_channel])
        self.servo_front_shoulder_right.set_pulse_width_range(min_pulse=self.servo_front_shoulder_right_min_pulse, max_pulse=self.servo_front_shoulder_right_max_pulse)

        if self.servo_front_leg_right_pca9685 == 1:
            self.servo_front_leg_right = servo.Servo(self.pca9685_1.channels[self.servo_front_leg_right_channel])
        else:
            self.servo_front_leg_right = servo.Servo(
                self.pca9685_2.channels[self.servo_front_leg_right_channel])
        self.servo_front_leg_right.set_pulse_width_range(min_pulse=self.servo_front_leg_right_min_pulse, max_pulse=self.servo_front_leg_right_max_pulse)

        if self.servo_front_feet_right_pca9685 == 1:
            self.servo_front_feet_right = servo.Servo(self.pca9685_1.channels[self.servo_front_feet_right_channel])
        else:
            self.servo_front_feet_right = servo.Servo(self.pca9685_2.channels[self.servo_front_feet_right_channel])
        self.servo_front_feet_right.set_pulse_width_range(min_pulse=self.servo_front_feet_right_min_pulse, max_pulse=self.servo_front_feet_right_max_pulse)

        if self.servo_arm_rotation_pca9685:

            if self.servo_arm_rotation_pca9685 == 1:
                self.servo_arm_rotation = servo.Servo(self.pca9685_1.channels[self.servo_arm_rotation_channel])
            else:
                self.servo_arm_rotation = servo.Servo(self.pca9685_2.channels[self.servo_arm_rotation_channel])
            self.servo_arm_rotation.set_pulse_width_range(min_pulse=self.servo_arm_rotation_min_pulse, max_pulse=self.servo_arm_rotation_max_pulse)

            if self.servo_arm_lift_pca9685 == 1:
                self.servo_arm_lift = servo.Servo(self.pca9685_1.channels[self.servo_arm_lift_channel])
            else:
                self.servo_arm_lift = servo.Servo(self.pca9685_2.channels[self.servo_arm_lift_channel])
            self.servo_arm_lift.set_pulse_width_range(min_pulse=self.servo_arm_lift_min_pulse, max_pulse=self.servo_arm_lift_max_pulse)

            if self.servo_arm_range_pca9685 == 1:
                self.servo_arm_range = servo.Servo(self.pca9685_1.channels[self.servo_arm_range_channel])
            else:
                self.servo_arm_range = servo.Servo(self.pca9685_2.channels[self.servo_arm_range_channel])
            self.servo_arm_range.set_pulse_width_range(min_pulse=self.servo_arm_range_min_pulse, max_pulse=self.servo_arm_range_max_pulse)

            if self.servo_arm_cam_tilt_pca9685 == 1:
                self.servo_arm_cam_tilt = servo.Servo(self.pca9685_1.channels[self.servo_arm_cam_tilt_channel])
            else:
                self.servo_arm_cam_tilt = servo.Servo(self.pca9685_2.channels[self.servo_arm_cam_tilt_channel])
            self.servo_arm_cam_tilt.set_pulse_width_range(min_pulse=self.servo_arm_cam_tilt_min_pulse, max_pulse=self.servo_arm_cam_tilt_max_pulse)

    def move(self):

        try:
            self.servo_rear_shoulder_left.angle = self.servo_rear_shoulder_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_shoulder_left angle requested')

        try:
            self.servo_rear_leg_left.angle = self.servo_rear_leg_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_leg_left angle requested')

        try:
            self.servo_rear_feet_left.angle = self.servo_rear_feet_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_feet_left angle requested')

        try:
            self.servo_rear_shoulder_right.angle = self.servo_rear_shoulder_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_shoulder_right angle requested')

        try:
            self.servo_rear_leg_right.angle = self.servo_rear_leg_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_leg_right angle requested')

        try:
            self.servo_rear_feet_right.angle = self.servo_rear_feet_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_rear_feet_right angle requested')

        try:
            self.servo_front_shoulder_left.angle = self.servo_front_shoulder_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_shoulder_left angle requested')

        try:
            self.servo_front_leg_left.angle = self.servo_front_leg_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_leg_left angle requested')

        try:
            self.servo_front_feet_left.angle = self.servo_front_feet_left_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_feet_left angle requested')

        try:
            self.servo_front_shoulder_right.angle = self.servo_front_shoulder_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_shoulder_right angle requested')

        try:
            self.servo_front_leg_right.angle = self.servo_front_leg_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_leg_right angle requested')

        try:
            self.servo_front_feet_right.angle = self.servo_front_feet_right_rest_angle
        except ValueError as e:
            log.error('Impossible servo_front_feet_right angle requested')

        if self.servo_arm_rotation_pca9685:
            try:
                self.servo_arm_rotation.angle = self.servo_arm_rotation_rest_angle
            except ValueError as e:
                log.error('Impossible servo_arm_rotation angle requested')

            try:
                self.servo_arm_lift.angle = self.servo_arm_lift_rest_angle
            except ValueError as e:
                log.error('Impossible arm_lift angle requested')

            try:
                self.servo_arm_range.angle = self.servo_arm_range_rest_angle
            except ValueError as e:
                log.error('Impossible servo_arm_range angle requested')

            try:
                self.servo_arm_cam_tilt.angle = self.servo_arm_cam_tilt_rest_angle
            except ValueError as e:
                log.error('Impossible servo_arm_cam_tilt angle requested')

    def auto_walking(self):
        # Walking Parameters
        b_height = 200
        h_amp = 100# horizontal step length
        v_amp = 40 #vertical step length
        track = 58.09
        x_offset = 0 #body offset in x direction 
        steering = 200 #Initial steering radius (arbitrary)
        walking_direction = 90/180*pi #Initial steering angle (arbitrary)
        stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2
        Angle = [0, 0]
        cw =1   
        continuer = True
        t = 0 #Initializing timing/clock
        tstart = 1 #End of start sequence
        tstop = 1000 #Start of stop sequence by default
        tstep = 0.01 #Timing/clock step
        tstep1 = tstep
        timing = [] #timing to plot distance
        
        walking = True #walking sequence activation
        stop = False # walking stop sequence activation
        cw = 1
        walking_speed = 0
        walking_direction = 0
        module = 0
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]
        
        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]
        # Caclulates/determines initial centre of gravity
        # Needed to find x,y,z spots below
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        # needed to find pos
        # pos updated ion loop to find radian for angle
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]
        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]
        
        while continuer == True:
            
            if self._motion_queue.empty():
                
                if (walking == True):  
                    coef = 1.0
                    
                tstep = tstep1
                
                if (1>0.2)|(1>0.2)|(stop == True):                
                    t=t+tstep
                    trec = int(t)+1
                    
                    
                    module_old = module
                    walking_direction_old = walking_direction
                    steering_old = steering
                    
                    x_old = module_old*cos(walking_direction_old)
                    y_old = module_old*sin(walking_direction_old)
                    
                    #update request
                    module = sqrt(1**2 + 1**2)
                    walking_direction = (atan2(-0,-0)%(2*pi)+pi/2)%(2*pi)
                    
                    x_new = module*cos(walking_direction)
                    y_new = module*sin(walking_direction)     
                    
                    # code for walking in a straight line. 
                    cw = 1
                    if (steering<2000):
                        steering = min(1e6,steering_old*coef) 
                    else:
                        steering = 1e6
                        
                    gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
                    
                    if (gap>0.01):
                        x_new = x_old+ (x_new-x_old)/gap*0.01
                        y_new = y_old+ (y_new-y_old)/gap*0.01
                        module = sqrt(x_new**2+y_new**2)
                        walking_direction = atan2(y_new,x_new)
                        
                                                                           
                    #reduces speed sideways and backwards  
                    min_h_amp = h_amp*(1/2e6*steering+1/2)               
                    xa = 1+cos(walking_direction-pi/2) 
                    walking_speed = min (1, module) * min(h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)
        
        
                if (1<0.2)&(1<0.2)&(stop == False):  
                    t=t+tstep                
                    module = max (0, module-0.01)
                    walking_speed = module* h_amp * ((1+cos(walking_direction-pi/2))/2*0.75+0.25)
                    if (steering<2000):
                        steering = min(1e6,steering*coef) 
                    else:
                        steering = 1e6
                    cw=1    
                    if (t>trec): 
                        t=trec
                        
                theta_spot[3] = Angle [0] # angle around x axis
                theta_spot[4] = Angle [1] # angle around y axis
                theta_spot[0] = Angle [0] # angle around x axis
                theta_spot[1] = Angle [1] # angle around y axis
                
                
                # updates pos
                if (t< tstart):           
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'start')
                else:
                    if (t<tstop):
                        pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'walk')
                    else:
                        pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'stop')    
                               
                theta_spot = pos[12]
                x_spot = pos[13]
                y_spot = pos[14]                 
                z_spot = pos[15]
                
                
                if (t>(tstop+1-tstep)):
                    stop = False
                    walking = False
                
                
                thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
                thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
                thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                
                # Updates CG - most likely not needed here as it is for animation and/or steering (both taken out)
                CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
                #Calculation of CG absolute position
                M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
                CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
                dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
                self.moving_feet(thetalf, thetarf, thetarr, thetalr, t, timing)
                
            else:
                
                event = self._motion_queue.get(block=True, timeout=60)
                if event['a']:
                    self.standing()
                    continuer = False
                    break
                
    def stop_turn(self, raw_value):
        # Walking Parameters
        b_height = 200
        h_amp = 100# horizontal step length
        v_amp = 40 #vertical step length
        track = 58.09
        x_offset = 0 #body offset in x direction 
        steering = 200 #Initial steering radius (arbitrary)
        walking_direction = 90/180*pi #Initial steering angle (arbitrary)
        stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2
        Angle = [0, 0]
        cw =1   
        continuer = True
        t = 0 #Initializing timing/clock
        tstart = 1 #End of start sequence
        tstop = 1000 #Start of stop sequence by default
        tstep = 0.01 #Timing/clock step
        timing = [] #timing to plot distance
        
        walking = True #walking sequence activation
        stop = False # walking stop sequence activation
        cw = 1
        walking_speed = 0
        walking_direction = 0
        module = 0
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]
        
        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]
        # Caclulates/determines initial centre of gravity
        # Needed to find x,y,z spots below
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        # needed to find pos
        # pos updated ion loop to find radian for angle
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]
        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]
        raw_value = int(raw_value)
        
        while continuer:
            if self._motion_queue.empty():
            
                coef = 1.2   
                   
                if (1>0.2)|(1>0.2)|(stop == True):                
                    t=t+tstep
                    trec = int(t)+1
                    
                    module_old = module
                    walking_direction_old = walking_direction
                    steering_old = steering
                    
                    x_old = module_old*cos(walking_direction_old)
                    y_old = module_old*sin(walking_direction_old)
                    
                    #update request
                    module = sqrt(1**2 + 1**2)
                    walking_direction = (atan2(-0,-0)%(2*pi)+pi/2)%(2*pi)
                    
                    x_new = module*cos(walking_direction)
                    y_new = module*sin(walking_direction) 
                        
                    #steering update                
                    if (abs(raw_value) < 0.2):
                        cw = 1
                        if (steering<2000):
                            steering = min(1e6,steering_old*coef) 
                        else:
                            steering = 1e6
                    else:
                        steering = 2000-(abs(raw_value)-0.2)*2000/0.8+0.001
                        if ((steering/steering_old)>coef):                       
                            steering = steering_old*coef
                        if ((steering_old/steering)>coef):                       
                            steering = steering_old/coef   
                            if (steering <0.001):
                                steering = 0.001
                        cw = -np.sign(raw_value)
                    
                    gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
                    
                    if (gap>0.01):
                        x_new = x_old+ (x_new-x_old)/gap*0.01
                        y_new = y_old+ (y_new-y_old)/gap*0.01
                        module = sqrt(x_new**2+y_new**2)
                        walking_direction = atan2(y_new,x_new)
                        
                                                                           
                    #reduces speed sideways and backwards  
                    min_h_amp = h_amp*(1/2e6*steering+1/2)               
                    xa = 1+cos(walking_direction-pi/2) 
                    walking_speed = min (1, module) * min(h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)
        
        
                if (1<0.2)&(1<0.2)&(stop == False):  
                    t=t+tstep                
                    module = max (0, module-0.01)
                    walking_speed = module* h_amp * ((1+cos(walking_direction-pi/2))/2*0.75+0.25)
                    if (steering<2000):
                        steering = min(1e6,steering*coef) 
                    else:
                        steering = 1e6
                    cw=1    
                    if (t>trec): 
                        t=trec
                        
                print(steering)
                        
                theta_spot[3] = Angle [0] # angle around x axis
                theta_spot[4] = Angle [1] # angle around y axis
                theta_spot[0] = Angle [0] # angle around x axis
                theta_spot[1] = Angle [1] # angle around y axis
                
                
                # updates pos
                if (t< tstart):           
                    pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'start')
                else:
                    if (t<tstop):
                        pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'walk')
                    else:
                        pos = Spot.start_walk_stop (track,x_offset,steering,walking_direction,cw,walking_speed,v_amp,b_height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,'stop')    
                               
                theta_spot = pos[12]
                x_spot = pos[13]
                y_spot = pos[14]                 
                z_spot = pos[15]
                
                
                if (t>(tstop+1-tstep)):
                    stop = False
                    walking = False
                
                
                thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
                thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
                thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                
                # Updates CG - most likely not needed here as it is for animation and/or steering (both taken out)
                CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
                #Calculation of CG absolute position
                M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
                CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
                dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
                print(raw_value)
                self.moving_feet(thetalf, thetarf, thetarr, thetalr, t, timing)
                
            else:
                
                event = self._motion_queue.get(block=True, timeout=60)
                if event['a']:
                    self.standing()
                    continuer = False
                    break
                
    def sit(self):
        b_height = 200
        track = 58.09
        x_offset = 0 #body offset in x direction 
        continuer = True
        t = 0 #Initializing timing/clock
        timing = [] #timing to plot distance
        tstep = 0.01 #Timing/clock step
        stop = False # walking stop sequence activation
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]
        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]
        # Caclulates/determines initial centre of gravity
        # Needed to find x,y,z spots below
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        # needed to find pos
        # pos updated ion loop to find radian for angle
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]
        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]
        
        while continuer == True:
            
            if self._motion_queue.empty():
                
                alpha_sitting = -30/180*pi 
                
                x_end_sitting = Spot.xlr-Spot.L2 + Spot.L1*cos(pi/3) +Spot.Lb/2*cos(-alpha_sitting) - Spot.d*sin (-alpha_sitting)
                z_end_sitting = Spot.L1*sin(pi/3)+ Spot.Lb/2*sin(-alpha_sitting) + Spot.d*cos(-alpha_sitting)
                start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations        
                
                #end_frame_pos = [0,0,0,x_offset,0,b_height-20] # x,y,z rotations then translations
                end_frame_pos = [0,alpha_sitting,0, x_end_sitting,0,z_end_sitting] # x,y,z rotations then translations
                pos = Spot.moving(t, start_frame_pos,end_frame_pos, pos)
                
                if (stop == False):
                    t=t+4*tstep
                    if (t>=1):
                        t= 1
                
                thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
                thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
                thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                
                # Updates CG - most likely not needed here as it is for animation and/or steering (both taken out)
                CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
                #Calculation of CG absolute position
                M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
                CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
                dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
                
                self.moving_feet(thetalf, thetarf, thetarr, thetalr, t, timing)
        
            else:
                event = self._motion_queue.get(block=True, timeout=60)
                if event['b']:
                    self.standing()
                    continuer = False
                    break
                
    def standing(self):
        # Walking Parameters
        b_height = 200
        track = 58.09
        x_offset = 0 #body offset in x direction 
        t = 0 #Initializing timing/clock
        timing = [] #timing to plot distance
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]
        
        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0] 
        # Caclulates/determines initial centre of gravity
        # Needed to find x,y,z spots below
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        # needed to find pos
        # pos updated ion loop to find radian for angle
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]
        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]

        self.moving_feet(thetalf, thetarf, thetarr, thetalr, t, timing)
        
    def pee(self):
        peeing = False
        shifting = True
        b_height = 200
        track = 58.09
        x_offset = 0 #body offset in x direction 
        ra_longi = 30 
        ra_lat = 30
        t = 1 #Initializing timing/clock
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
        z_spot = [0,b_height,0,0,0,0,0,0,0]
        theta_spot = [0,0,0,0,0,0]
        joype = -1
        continuer = True
        stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
        timing = []
        #theta xyz of ground then theta xyz of frame/body
        pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos_init[9], pos_init[10], pos_init[11], 1)[0]
        # Caclulates/determines initial centre of gravity
        # Needed to find x,y,z spots below
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        # needed to find pos
        # pos updated ion loop to find radian for angle
        x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,CG[0],CGabs[0],dCG[1]]
        y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,CG[1],CGabs[1],dCG[2]]
        z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]
        pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]
        
        while continuer == True:
            
            if self._motion_queue.empty():
        
                if (shifting == True):                        
                    x_end_shifting = ra_longi
                    y_end_shifting = -ra_lat
                    start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
                    end_frame_pos = [0,0,0, x_end_shifting+x_offset,y_end_shifting,b_height] # x,y,z rotations then translations
                    pos = Spot.moving (t, start_frame_pos,end_frame_pos, pos)
                    
                    if (t==1)&(peeing == False):
                        pos_shift_init = pos
                    
                    if (t == 1): #lifting left hid leg is possible 
                        
                        if (peeing == True): 
                            pos[9] = pos_shift_init[9]+ (0-pos_shift_init[9])*(joype+1)/2
                            pos[10] = pos_shift_init[10]+ (130-pos_shift_init[10])*(joype+1)/2
                            pos[11] = pos_shift_init[11]+ (-20-pos_shift_init[11])*(joype+1)/2
                            
                            thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                            #update of left hind leg absolute position                  
                            xleglr =Spot.xlr+pos[9]
                            yleglr =Spot.ylr+pos[10]
                            zleglr =pos[11]
                            theta_spot_shift = pos[12]
                            x_spot_shift = pos[13]
                            y_spot_shift = pos[14]
                            z_spot_shift = pos[15]
                            M = Spot.xyz_rotation_matrix(theta_spot_shift[3],theta_spot_shift[4],theta_spot_shift[2]+theta_spot_shift[5],False)
                            pee_lr = Spot.new_coordinates(M,xleglr,yleglr,zleglr,x_spot_shift[1],y_spot_shift[1],z_spot_shift[1])
                            
                            x_spot_shift[5] = pee_lr[0]
                            y_spot_shift[5] = pee_lr[1]
                            z_spot_shift[5] = pee_lr[2]
                            pos[13] = x_spot_shift
                            pos[14] = y_spot_shift
                            pos[15] = z_spot_shift
                            
                        if ((0.59827880859375 != 0)&(0.59827880859375  !=-1))|(joype != -1):
                            peeing = True
                            if (0.09827880859375>= joype):
                                 joype = min (0.59827880859375,joype+0.1)
                            else:
                                 joype = max (0.59827880859375,joype-0.1)
                        else:
                            peeing = False
                            
                thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
                thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
                thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
                thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]
                
                # Updates CG - most likely not needed here as it is for animation and/or steering (both taken out)
                CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
                #Calculation of CG absolute position
                M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
                CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
                dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
                            
                self.moving_feet(thetalf, thetarf, thetarr, thetalr, t, timing)
                
            else:
                event = self._motion_queue.get(block=True, timeout=60)
                if event['x']:
                    self.standing()
                    continuer = False
                    break
            
    def rest_position(self):

        self.servo_rear_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)
        self.servo_rear_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)
        self.servo_rear_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)
        self.servo_rear_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)
        self.servo_rear_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)
        self.servo_rear_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)
        self.servo_front_shoulder_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)
        self.servo_front_leg_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)
        self.servo_front_feet_left_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)
        self.servo_front_shoulder_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)
        self.servo_front_leg_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)
        self.servo_front_feet_right_rest_angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)

        if self.servo_arm_rotation_pca9685:
            self.servo_arm_rotation.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)
            self.servo_arm_lift.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)
            self.servo_arm_range.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)
            self.servo_arm_cam_tilt.angle = Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)

    def body_move_body_up_and_down(self, raw_value):

        range = 10
        range2 = 15

        if raw_value < 0:
            self.servo_rear_leg_left_rest_angle -= range
            self.servo_rear_feet_left_rest_angle += range2
            self.servo_rear_leg_right_rest_angle += range
            self.servo_rear_feet_right_rest_angle -= range2
            self.servo_front_leg_left_rest_angle -= range
            self.servo_front_feet_left_rest_angle += range2
            self.servo_front_leg_right_rest_angle += range
            self.servo_front_feet_right_rest_angle -= range2

        elif raw_value > 0:
            self.servo_rear_leg_left_rest_angle += range
            self.servo_rear_feet_left_rest_angle -= range2
            self.servo_rear_leg_right_rest_angle -= range
            self.servo_rear_feet_right_rest_angle += range2
            self.servo_front_leg_left_rest_angle += range
            self.servo_front_feet_left_rest_angle -= range2
            self.servo_front_leg_right_rest_angle -= range
            self.servo_front_feet_right_rest_angle += range2

        else:
            self.rest_position()

        print('body_move_body_up_and_down - ' + str(self.servo_rear_leg_left_rest_angle) + ', ' + str(self.servo_rear_feet_left_rest_angle) + ', ' + str(self.servo_rear_leg_right_rest_angle) + ', ' + str(self.servo_rear_feet_right_rest_angle) + ', ' + str(self.servo_front_leg_left_rest_angle) + ', ' + str(self.servo_front_feet_left_rest_angle) + ', ' + str(self.servo_front_leg_right_rest_angle) + ', ' + str(self.servo_front_feet_right_rest_angle))

    def body_move_body_up_and_down_analog(self, raw_value):

        servo_rear_leg_left_max_angle = 38
        servo_rear_feet_left_max_angle = 70
        servo_rear_leg_right_max_angle = 126
        servo_rear_feet_right_max_angle = 102
        servo_front_leg_left_max_angle = 57
        servo_front_feet_left_max_angle = 85
        servo_front_leg_right_max_angle = 130
        servo_front_feet_right_max_angle = 120

        delta_rear_leg_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE), servo_rear_leg_left_max_angle), raw_value))
        delta_rear_feet_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE), servo_rear_feet_left_max_angle), raw_value))
        delta_rear_leg_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE), servo_rear_leg_right_max_angle), raw_value))
        delta_rear_feet_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE), servo_rear_feet_right_max_angle), raw_value))
        delta_front_leg_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE), servo_front_leg_left_max_angle), raw_value))
        delta_front_feet_left = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE), servo_front_feet_left_max_angle), raw_value))
        delta_front_leg_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE), servo_front_leg_right_max_angle), raw_value))
        delta_front_feet_right = int(General().maprange((1, -1), (Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE), servo_front_feet_right_max_angle), raw_value))

        self.servo_rear_leg_left_rest_angle = delta_rear_leg_left
        self.servo_rear_feet_left_rest_angle = delta_rear_feet_left
        self.servo_rear_leg_right_rest_angle = delta_rear_leg_right

        self.servo_rear_feet_right_rest_angle = delta_rear_feet_right
        self.servo_front_leg_left_rest_angle = delta_front_leg_left
        self.servo_front_feet_left_rest_angle = delta_front_feet_left
        self.servo_front_leg_right_rest_angle = delta_front_leg_right
        self.servo_front_feet_right_rest_angle = delta_front_feet_right

        print('body_move_body_up_and_down_analog - ' + str(self.servo_rear_leg_left_rest_angle) + ', ' + str(self.servo_rear_feet_left_rest_angle) + ', ' + str(self.servo_rear_leg_right_rest_angle) + ', ' + str(self.servo_rear_feet_right_rest_angle) + ', ' + str(self.servo_front_leg_left_rest_angle) + ', ' + str(self.servo_front_feet_left_rest_angle) + ', ' + str(self.servo_front_leg_right_rest_angle) + ', ' + str(self.servo_front_feet_right_rest_angle))

    def body_move_body_left_right(self, raw_value):

        range = 5

        if raw_value < 0:
            self.servo_rear_shoulder_left_rest_angle -= range
            self.servo_rear_shoulder_right_rest_angle -= range
            self.servo_front_shoulder_left_rest_angle += range
            self.servo_front_shoulder_right_rest_angle += range

        elif raw_value > 0:
            self.servo_rear_shoulder_left_rest_angle += range
            self.servo_rear_shoulder_right_rest_angle += range
            self.servo_front_shoulder_left_rest_angle -= range
            self.servo_front_shoulder_right_rest_angle -= range

        else:
            self.rest_position()

    def body_move_body_left_right_analog(self, raw_value):

        delta_a = int(General().maprange((-1, 1), (30, 100), raw_value))
        delta_b = int(General().maprange((-1, 1), (100, 30), raw_value))

        self.servo_rear_shoulder_left_rest_angle = delta_a
        self.servo_rear_shoulder_right_rest_angle = delta_a
        self.servo_front_shoulder_left_rest_angle = delta_b
        self.servo_front_shoulder_right_rest_angle = delta_b

    def standing_position(self):

        variation_leg = 50
        variation_feet = 70

        self.servo_rear_shoulder_left.angle = self.servo_rear_shoulder_left_rest_angle + 10
        self.servo_rear_leg_left.angle = self.servo_rear_leg_left_rest_angle - variation_leg
        self.servo_rear_feet_left.angle = self.servo_rear_feet_left_rest_angle + variation_feet

        
        self.servo_rear_shoulder_right.angle = self.servo_rear_shoulder_right_rest_angle - 10
        self.servo_rear_leg_right.angle = self.servo_rear_leg_right_rest_angle + variation_leg
        self.servo_rear_feet_right.angle = self.servo_rear_feet_right_rest_angle - variation_feet

        time.sleep(0.05)

        self.servo_front_shoulder_left.angle = self.servo_front_shoulder_left_rest_angle - 10
        self.servo_front_leg_left.angle = self.servo_front_leg_left_rest_angle - variation_leg + 5
        self.servo_front_feet_left.angle = self.servo_front_feet_left_rest_angle + variation_feet - 5

        self.servo_front_shoulder_right.angle = self.servo_front_shoulder_right_rest_angle + 10
        self.servo_front_leg_right.angle = self.servo_front_leg_right_rest_angle + variation_leg - 5
        self.servo_front_feet_right.angle = self.servo_front_feet_right_rest_angle - variation_feet + 5

    def body_move_position_left(self):

        move = 20

        variation_leg = 50
        variation_feet = 70

        self.servo_rear_shoulder_left.angle = self.servo_rear_shoulder_left_rest_angle + 10 - move
        self.servo_rear_leg_left.angle = self.servo_rear_leg_left_rest_angle - variation_leg

        self.servo_rear_feet_left.angle = self.servo_rear_feet_left_rest_angle + variation_feet

        self.servo_rear_shoulder_right.angle = self.servo_rear_shoulder_right_rest_angle - 10 - move
        self.servo_rear_leg_right.angle = self.servo_rear_leg_right_rest_angle + variation_leg
        self.servo_rear_feet_right.angle = self.servo_rear_feet_right_rest_angle - variation_feet

        time.sleep(0.05)

        self.servo_front_shoulder_left.angle = self.servo_front_shoulder_left_rest_angle - 10 + move
        self.servo_front_leg_left.angle = self.servo_front_leg_left_rest_angle - variation_leg + 5
        self.servo_front_feet_left.angle = self.servo_front_feet_left_rest_angle + variation_feet - 5

        self.servo_front_shoulder_right.angle = self.servo_front_shoulder_right_rest_angle + 10 + move
        self.servo_front_leg_right.angle = self.servo_front_leg_right_rest_angle + variation_leg - 5
        self.servo_front_feet_right.angle = self.servo_front_feet_right_rest_angle - variation_feet + 5

    def arm_set_rotation(self, raw_value):

        if not self.servo_arm_rotation_pca9685:
            return

        left_position = int(General().maprange((-1, 1), (0, 180), raw_value / 2))

        if int(self.servo_arm_rotation.angle) != int(left_position):
            self.servo_arm_rotation.angle = left_position

    def arm_set_lift(self, raw_value):

        if not self.servo_arm_rotation_pca9685:
            return

        lift_position = int(General().maprange((-1, 1), (180, 0), raw_value / 2))

        if int(self.servo_arm_lift.angle) != int(lift_position):
            self.servo_arm_lift.angle = lift_position

    def arm_set_range(self, raw_value):

        if not self.servo_arm_rotation_pca9685:
            return

        range_position = int(General().maprange((-1, 1), (180, 0), raw_value / 2))

        if int(self.servo_arm_range.angle) != int(range_position):
            self.servo_arm_range.angle = range_position

    def arm_set_cam_tilt(self, raw_value):

        if not self.servo_arm_rotation_pca9685:
            return

        tilt_position = int(General().maprange((-1, 1), (100, 150), raw_value))

        if int(self.servo_arm_cam_tilt.angle) != int(tilt_position):
            self.servo_arm_cam_tilt.angel = tilt_position
        
    def moving_feet(self, thetalf, thetarf, thetarr, thetalr, t, timing):
        
        # front left angle
        delta_front_shoulder_left = 180 - (int(thetalf[0] * (180/pi)) + 90)
        #print("Shoulder_front_left:", delta_front_shoulder_left)
        
        delta_front_leg_left = int(thetalf[1] * (180/pi)) + 90
        #print("Leg_front_left:", delta_front_leg_left)
        
        delta_front_feet_left = int(thetalf[2] * (180/pi)) + 140
        #print("Feet_front_left:", delta_front_feet_left)
        
        # front right angle
        delta_front_shoulder_right = 180 - (int(thetarf[0] * (180/pi)) + 90)
        #print("Shoulder_front_right:", delta_front_shoulder_right)
    
        delta_front_leg_right = 180 - (int(thetarf[1] * (180/pi)) + 90)
        #print("Leg_front_right:", delta_front_leg_right)
        
        delta_front_feet_right = 180 - (int(thetarf[2] * (180/pi)) + 140)
        #print("Feet_front_right:", delta_front_feet_right)
        
        # rear left angle
        delta_rear_shoulder_left = int(thetalr[0] * (180/pi)) + 90
        #print("Shoulder_rear_left:", delta_rear_shoulder_left)
        
        delta_rear_leg_left = int(thetalr[1] * (180/pi)) + 90
        #print("Leg_rear_left:", delta_rear_leg_left)
        
        delta_rear_feet_left = int(thetalr[2] * (180/pi)) + 150
        #print("Feet_rear_left:", delta_rear_feet_left)
        
        # rear right angle
        delta_rear_shoulder_right = int(thetarr[0] * (180/pi)) + 90
        #print("Shoulder_rear_right:", delta_rear_shoulder_right)
        
        delta_rear_leg_right = 180 - (int(thetarr[1] * (180/pi)) + 90)
        #print("Leg_rear_right:", delta_rear_leg_right)
        
        delta_rear_feet_right = 180 - (int(thetarr[2] * (180/pi)) + 150)
        #print("Feet_rear_right:", delta_rear_feet_right)
        
        # front left movement
        self.servo_front_shoulder_left_rest_angle = delta_front_shoulder_left
        self.servo_front_leg_left_rest_angle = delta_front_leg_left
        self.servo_front_feet_left_rest_angle = delta_front_feet_left
        
        
        # front right movement
        self.servo_front_shoulder_right_rest_angle = delta_front_shoulder_right
        self.servo_front_leg_right_rest_angle = delta_front_leg_right
        self.servo_front_feet_right_rest_angle = delta_front_feet_right
        
        
        # rear left movement
        self.servo_rear_shoulder_left_rest_angle = delta_rear_shoulder_left
        self.servo_rear_leg_left_rest_angle = delta_rear_leg_left
        self.servo_rear_feet_left_rest_angle = delta_rear_feet_left
        
        # rear right movement
        self.servo_rear_shoulder_right_rest_angle = delta_rear_shoulder_right
        self.servo_rear_leg_right_rest_angle = delta_rear_leg_right
        self.servo_rear_feet_right_rest_angle = delta_rear_feet_right
            
        timing.append(t) 
        self.move()
        time.sleep(0.002)