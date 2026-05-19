import math
import threading
import time
from pathlib import Path
import csv
from datetime import datetime
from lbr_fri_idl.msg import LBRState
#from build.lbr_fri_idl.ament_cmake_python.lbr_fri_idl.lbr_fri_idl.msg._lbr_state import LBRState
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_srvs.srv import SetBool

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

BASE_LINK       = 'lbr_link_0'
END_EFFECTOR    = 'lbr_link_ee'
GROUP_NAME      = 'arm'

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

MAX_VELOCITY     = 0.15
MAX_ACCELERATION = 0.15
WAIT_TIMEOUT     = 10.0   # Seconds for joint-state and TF wait time

# I/O Configuration

OUTPUT_CHANNELS = 4
OUTPUT_SERVICE_NAME_TEMPLATE = '/lbr/digital_output/ch{channel}/set'
DIGITAL_OUTPUT_TIMEOUT = 5.0
DIGITAL_OUTPUT_CONTINUE_WAIT = 1.0
DIGITAL_OUTPUT_REQUIRED = False
DATA_RECORDING_SETTLE_TIME = 0.2
DATA_RECORDING_SAMPLE_TIMEOUT = 1.0

# -----------------------------------------------------------------------------
# Digital Output Client
# -----------------------------------------------------------------------------

class DigitalOutputClient:
    '''General client for switching digital outputs.'''

    def __init__(
        self,
        node: Node,
        callback_group,
        num_channels: int = 4,
        service_name_template: str = '/lbr/digital_output/ch{channel}/set',
    ):
        self._node = node
        self._num_channels = num_channels
        self._service_name_template = service_name_template
        self._states: dict[int, bool] = {ch: False for ch in range(1, num_channels + 1)}
        self._clients = {
            ch: self._node.create_client(
                SetBool,
                self._service_name(channel=ch),
                callback_group=callback_group,
            )
            for ch in self._states
        }

    def wait_for_service(self, channel: int, timeout_sec: float = 5.0, log_error: bool = True) -> bool:
        '''Wait until the digital-output service is available.'''
        client = self._clients.get(channel)
        if client is None:
            if log_error:
                self._node.get_logger().error(f'Digital-Output ch={channel} Service client is missing')
            return False
        service_name = self._service_name(channel)
        self._node.get_logger().info(f'Waiting for digital-output service: {service_name}')
        if not client.wait_for_service(timeout_sec=timeout_sec):
            if log_error:
                self._node.get_logger().error(
                    f'Digital-Output ch={channel} Service not available: {service_name}'
                )
            return False
        return True

    def set_output(self, channel: int, state: bool, timeout_sec: float = 2.0) -> bool:
        '''Switches a digital output.'''
        if channel not in self._states:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} does not exist. Valid channels: 1..{self._num_channels}'
            )
            return False

        self._node.get_logger().info(
            f'Digital-Output ch={channel} → {"ON" if state else "OFF"}'
        )
        if not self._call_service(channel, state, timeout_sec=timeout_sec):
            return False

        self._states[channel] = state
        return True

    def get_output(self, channel: int) -> bool | None:
        '''Returns the last set state of a channel.'''
        return self._states.get(channel)

    def _service_name(self, channel: int) -> str:
        return self._service_name_template.format(channel=channel)

    def _call_service(self, channel: int, state: bool, timeout_sec: float = 2.0) -> bool:
        client = self._clients.get(channel)
        if client is None:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service client is missing'
            )
            return False

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service not available: {self._service_name(channel)}'
            )
            return False

        req = SetBool.Request()
        req.data = state
        future = client.call_async(req)

        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)

        if not future.done():
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service-Timeout'
            )
            return False

        try:
            resp = future.result()
        except Exception as exc:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service error: {exc}'
            )
            return False

        if resp is None:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service error'
            )
            return False

        if not resp.success:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} not switched: {resp.message}'
            )
        return resp.success

# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place_iiwa14')

        self.callback_group = ReentrantCallbackGroup()
        self._joint_state_received = False

        self.create_subscription(JointState, 'joint_states', self._on_joint_state, 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP_NAME,
            callback_group=self.callback_group,
            use_move_group_action=False,
            ignore_new_calls_while_executing=True,
        )
        self.moveit2.max_velocity     = MAX_VELOCITY
        self.moveit2.max_acceleration = MAX_ACCELERATION

        # Digital Output Client
        self.digital_output_timeout = DIGITAL_OUTPUT_TIMEOUT
        self.digital_output_continue_wait = DIGITAL_OUTPUT_CONTINUE_WAIT
        self.digital_output_required = DIGITAL_OUTPUT_REQUIRED
        self.digital_out = DigitalOutputClient(
            node=self,
            callback_group=self.callback_group,
            num_channels=OUTPUT_CHANNELS,
            service_name_template=OUTPUT_SERVICE_NAME_TEMPLATE,
        )

        # Log CSV file
        self._current_step = 'init'
        self._axis_torque = [0.00] * 7
        self._last_lbr_state_time = 0.0
        self._data_recording_lock = threading.Lock()
        self.data_recording_settle_time = DATA_RECORDING_SETTLE_TIME
        self.data_recording_sample_timeout = DATA_RECORDING_SAMPLE_TIMEOUT
        self.create_subscription(LBRState, '/lbr/lbr_state', self._on_lbr_state, 10)

        log_dir  = Path.cwd() / 'log' / 'torque_logs'
        Path.mkdir(log_dir, parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_file_path = log_dir / f'torque_{timestamp}.csv'
        self._data_recording_csv_file_path = log_dir / f'data_recording_{timestamp}.csv'

        with open(self._csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'step'] + [f'A_{i+1}' for i in range(7)])
        self.get_logger().info(f'Touque-Log: {self._csv_file_path}')

        with open(self._data_recording_csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['timestamp', 'step'] + [f'A_{i+1}' for i in range(7)]
            )
        self.get_logger().info(f'Data-recording torque log: {self._data_recording_csv_file_path}')

    # -------------------------------------------------------------------------
    # Callbacks & Helper Methods
    # -------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        if not self._joint_state_received:
            self.get_logger().info('Joint States available.')
        self._joint_state_received = True

    def _wait_for(self, condition_fn, label: str, timeout: float = WAIT_TIMEOUT) -> bool:
        '''Waits until condition_fn() returns True or a timeout occurs.'''
        self.get_logger().info(f'Wait for: {label}')
        deadline = time.time() + timeout
        while rclpy.ok():
            if condition_fn():
                return True
            if time.time() > deadline:
                self.get_logger().error(f'Timeout: {label}')
                return False
            time.sleep(0.05)
        return False

    def _wait_for_joint_states(self) -> bool:
        return self._wait_for(
            lambda: self._joint_state_received,
            'Joint States',
        )

    def _wait_for_tf(self) -> bool:
        def tf_available():
            try:
                self.tf_buffer.lookup_transform(
                    BASE_LINK, END_EFFECTOR,
                    rclpy.time.Time(), timeout=Duration(seconds=0.2),
                )
                self.get_logger().info('TF available.')
                return True
            except Exception:
                return False

        return self._wait_for(tf_available, f'TF {BASE_LINK} → {END_EFFECTOR}')

    def _get_ee_pose(self) -> tuple[list[float], list[float]]:
        '''
        Reads current end effector pose relative to BASE_LINK.

        Returns:
            position:  [x, y, z]
            quat_xyzw: [x, y, z, w]
        '''
        t = self.tf_buffer.lookup_transform(
            BASE_LINK, END_EFFECTOR,
            rclpy.time.Time(), timeout=Duration(seconds=2.0),
        )
        pos  = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        quat = [t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w]
        return pos, quat

    def _apply_rotation_absolut(self, current_quat: list[float], roll: float = None, pitch: float = None,  yaw: float = None) -> list[float]:
        '''Sets angles to absolute values ​​– missing axes from current orientation.'''
        if not any(v is not None for v in (roll, pitch, yaw)):
            return current_quat
        current_rpy = Rotation.from_quat(current_quat).as_euler('xyz')
        r = roll  if roll  is not None else current_rpy[0]
        p = pitch if pitch is not None else current_rpy[1]
        y = yaw   if yaw   is not None else current_rpy[2]
        return Rotation.from_euler('xyz', [r, p, y]).as_quat().tolist()

    def _apply_rotation_relativ(self, current_quat: list[float], roll: float = 0.0, pitch: float = 0.0,  yaw: float = 0.0) -> list[float]:
        '''Adds angles to the current orientation.'''
        current_rpy = Rotation.from_quat(current_quat).as_euler('xyz')
        r = current_rpy[0] + roll
        p = current_rpy[1] + pitch
        y = current_rpy[2] + yaw
        return Rotation.from_euler('xyz', [r, p, y]).as_quat().tolist()

    def _on_lbr_state(self, msg: LBRState):
        axis_torque = list(msg.external_torque)
        with self._data_recording_lock:
            self._axis_torque = axis_torque
            self._last_lbr_state_time = time.time()
        self._log_torque(self._current_step, axis_torque)

    def _log_torque(self, step: str, axis_torque: list[float]):
        row = [f'{time.time():.3f}', step] + [f'{t:.4f}' for t in axis_torque]
        with open(self._csv_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow(row)

    def record_data_point(self, step: str) -> bool:
        if self.data_recording_settle_time > 0.0:
            time.sleep(self.data_recording_settle_time)

        request_time = time.time()
        deadline = request_time + self.data_recording_sample_timeout
        axis_torque = None

        while rclpy.ok() and time.time() < deadline:
            with self._data_recording_lock:
                if self._last_lbr_state_time >= request_time:
                    axis_torque = list(self._axis_torque)
                    break
            time.sleep(0.01)

        if axis_torque is None:
            self.get_logger().error(f'No current LBRState sample received for {step}')
            return False

        row = [f'{time.time():.3f}', step] + [f'{value:.4f}' for value in axis_torque]
        with open(self._data_recording_csv_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(f'[{step}] current data-recording values saved')
        return True

    # -------------------------------------------------------------------------
    # Movement Commands
    # -------------------------------------------------------------------------

    def move_to_joint_position(self, joint_positions: list[float], name: str = ''):
        '''Moves to a joint position (values ​​in radians).'''
        # set log step for torque logging
        self._current_step = name

        self.get_logger().info(
            f'[{name}] joint position = '
            + ', '.join(f'{j}={v:.3f}' for j, v in zip(JOINT_NAMES, joint_positions))
        )
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'[{name}] ✓ complete')

    def move_to_base_position(self, x: float, y: float, z: float, roll: float = None, pitch: float = None, yaw: float = None, name: str = ''):
        '''Position and Rotation absolute in the BASE_LINK-Frame.'''
        # set log step for torque logging
        self._current_step = name

        _, current_quat = self._get_ee_pose()
        target_quat = self._apply_rotation_absolut(current_quat, roll, pitch, yaw)

        self.get_logger().info(f'[{name}] Absolute position = ({x:.3f}, {y:.3f}, {z:.3f})')
        self.moveit2.move_to_pose(position=[x, y, z], quat_xyzw=target_quat, cartesian=False)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'[{name}] ✓ complete')

    def move_to_relative_position(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, name: str = ''):
        '''Position and Rotation relative to the current EE pose.'''
        # set log step for torque logging
        self._current_step = name

        current_pos, current_quat = self._get_ee_pose()
        target = [current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz]
        target_quat = self._apply_rotation_relativ(current_quat, roll, pitch, yaw)

        self.get_logger().info(
            f'[{name}] Relativ Δpos = ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  '
            f'Δrot = ({math.degrees(roll):+.1f}°,{math.degrees(pitch):+.1f}°,{math.degrees(yaw):+.1f}°)'
            f'Current-Pos = ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}), Current-Rot = ({math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[0]):.1f}°,{math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[1]):.1f}°,{math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[2]):.1f}°)'
        )
        self.moveit2.move_to_pose(position=target, quat_xyzw=target_quat, cartesian=False)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'[{name}] ✓ complete')

    # -------------------------------------------------------------------------
    # I/O Commands
    # -------------------------------------------------------------------------

    def set_digital_output(self, channel: int, state: bool) -> bool:
        '''Switches one digital output through the robot I/O service.'''
        label = 'ON' if state else 'OFF'

        self.get_logger().info(f'Switch digital output ch={channel} → {label}')
        switched = self.digital_out.set_output(
            channel=channel,
            state=state,
            timeout_sec=self.digital_output_timeout,
        )
        if switched:
            return True

        if self.digital_output_required:
            return False

        self.get_logger().warn(
            f'Digital output ch={channel} was not switched. '
            f'Continuing after {self.digital_output_continue_wait:.1f}s.'
        )
        time.sleep(self.digital_output_continue_wait)
        return True

    # -------------------------------------------------------------------------
    # Main process
    # -------------------------------------------------------------------------

    def run_job(self) -> int:
        self.get_logger().info('=== Pick & Place launched ===')

        #if not self._wait_for_joint_states():
        #    return 1
        #if not self._wait_for_tf():
        #    return 1
        #self._current_step = 'home'
        self.move_to_joint_position(HOME_POSITION, 'home')
        #self._current_step = 'start'
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'pick'
        self.move_to_joint_position(PICK_JOINT_POSITION, 'pick')
        #self._current_step = 'pick-down'
        self.move_to_relative_position(dz=-0.10, name='pick-down')
        # -------
        if not self.set_digital_output(3, True):
            return 1
        # -------
        #self._current_step = 'pick-up'
        self.move_to_relative_position(dz=0.10, name='pick-up')
        # -------
        #self._current_step = 'start'
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'data_recording_front'
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(0),
            pitch=math.radians(30),
            name='data_recording_front'
        )
        self.record_data_point('data_recording_front')
        # -------
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'data_recording_left'
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(-30),
            pitch=math.radians(0),
            name='data_recording_left'
        )
        self.record_data_point('data_recording_left')
        # -------
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'data_recording_right'
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(0),
            pitch=math.radians(-30),
            name='data_recording_right'
        )
        self.record_data_point('data_recording_right')
        # -------
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'data_recording_back'
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(30),
            pitch=math.radians(0),
            name='data_recording_back'
        )
        self.record_data_point('data_recording_back')
        # -------
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'start'
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self._current_step = 'place-down'
        self.move_to_relative_position(dz=-0.10, name='place-down')
        # -------
        if not self.set_digital_output(3, False):
            return 1
        # -------
        #self._current_step = 'place-up'
        self.move_to_relative_position(dz=0.10, name='place-up')
        #self._current_step = 'home'
        self.move_to_joint_position(HOME_POSITION, 'home')

        self.get_logger().info('=== Pick & Place successfully completed ===')
        return 0


# -----------------------------------------------------------------------------
# Entry Point
# -----------------------------------------------------------------------------

def main() -> int:
    rclpy.init()
    node = PickPlace()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    exit_code = 1
    try:
        exit_code = node.run_job()
    except KeyboardInterrupt:
        node.get_logger().warn('Aborted.')
        exit_code = 130
    except Exception as exc:
        node.get_logger().error(f'Error: {exc}')
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
