import math

BASE_LINK = 'lbr_link_0'
END_EFFECTOR = 'lbr_link_ee'
GROUP_NAME = 'arm'

JOINT_NAMES = ['lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7']

HOME_POSITION = [0.0] * 7

START_JOINT_POSITION = [
    0.0,
    math.radians(15),
    0.0,
    math.radians(-90),
    0.0,
    math.radians(75),
    0.0,
]

PICK_JOINT_POSITION = [
    math.radians(30),
    math.radians(15),
    0.0,
    math.radians(-90),
    0.0,
    math.radians(75),
    0.0,
]

MAX_VELOCITY = 0.15
MAX_ACCELERATION = 0.15
WAIT_TIMEOUT = 10.0

OUTPUT_CHANNELS = 4
OUTPUT_SERVICE_NAME_TEMPLATE = '/lbr/digital_output/ch{channel}/set'
DIGITAL_OUTPUT_TIMEOUT = 5.0
DIGITAL_OUTPUT_CONTINUE_WAIT = 1.0
DIGITAL_OUTPUT_REQUIRED = False

DATA_RECORDING_SETTLE_TIME = 0.2
DATA_RECORDING_SAMPLE_TIMEOUT = 1.0
