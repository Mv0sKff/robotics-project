import math
import threading
import time

from lbr_fri_idl.msg import LBRState
#from build.lbr_fri_idl.ament_cmake_python.lbr_fri_idl.lbr_fri_idl.msg._lbr_state import LBRState
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from pymoveit2 import MoveIt2, MoveIt2State
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

from robotik_projekt.digital_output import DigitalOutputClient
from robotik_projekt.pick_place_iiwa14_config import (
    BASE_LINK,
    DATA_RECORDING_SAMPLE_TIMEOUT,
    DATA_RECORDING_SETTLE_TIME,
    DIGITAL_OUTPUT_CONTINUE_WAIT,
    DIGITAL_OUTPUT_REQUIRED,
    DIGITAL_OUTPUT_TIMEOUT,
    END_EFFECTOR,
    GROUP_NAME,
    HOME_POSITION,
    JOINT_NAMES,
    MAX_ACCELERATION,
    MAX_VELOCITY,
    OUTPUT_CHANNELS,
    OUTPUT_SERVICE_NAME_TEMPLATE,
    PICK_JOINT_POSITION,
    START_JOINT_POSITION,
    WAIT_TIMEOUT,
)
from robotik_projekt.torque_logger import TorqueLogger

# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place_iiwa14')

        self.callback_group = ReentrantCallbackGroup()
        self._joint_state_received = False
        self._joint_state = None

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

        self.torque_logger = TorqueLogger(
            node=self,
            data_recording_settle_time=DATA_RECORDING_SETTLE_TIME,
            data_recording_sample_timeout=DATA_RECORDING_SAMPLE_TIMEOUT,
        )
        self.create_subscription(LBRState, '/lbr/lbr_state', self._on_lbr_state, 10)

    # -------------------------------------------------------------------------
    # Callbacks & Helper Methods
    # -------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        if not self._joint_state_received:
            self.get_logger().info('Joint States available.')
        self._joint_state = msg
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
        self.torque_logger.on_lbr_state(msg)

    def record_data_point(self, step: str) -> bool:
        return self.torque_logger.record_data_point(step)

    # -------------------------------------------------------------------------
    # Movement Commands
    # -------------------------------------------------------------------------

    def _plan_and_execute(self, name: str, **planning_args) -> bool:
        """Plan and execute a movement using the background executor."""
        start_joint_state = self._joint_state
        if start_joint_state is None:
            self.get_logger().error(f'[{name}] no joint state available for planning')
            return False

        try:
            planning_future = self.moveit2.plan_async(
                start_joint_state=start_joint_state,
                **planning_args,
            )
            if planning_future is None:
                self.get_logger().error(f'[{name}] planning request could not be sent')
                return False

            while rclpy.ok() and not planning_future.done():
                time.sleep(0.01)

            if not planning_future.done():
                self.get_logger().error(f'[{name}] planning interrupted')
                return False

            trajectory = self.moveit2.get_trajectory(
                planning_future,
                cartesian=planning_args.get('cartesian', False),
            )
            if trajectory is None:
                self.get_logger().error(f'[{name}] planning failed')
                return False

            self.moveit2.execute(trajectory)
            if self.moveit2.query_state() == MoveIt2State.IDLE:
                self.get_logger().error(f'[{name}] trajectory execution could not be started')
                return False

            while (
                rclpy.ok()
                and self.moveit2.query_state() != MoveIt2State.IDLE
            ):
                time.sleep(0.01)

            return rclpy.ok() and self.moveit2.motion_suceeded
        except Exception as exc:
            self.get_logger().error(f'[{name}] movement error: {exc}')
            return False

    def move_to_joint_position(self, joint_positions: list[float], name: str = ''):
        '''Moves to a joint position (values ​​in radians).'''
        # set log step for torque logging
        self.torque_logger.set_step(name)

        self.get_logger().info(
            f'[{name}] joint position = '
            + ', '.join(f'{j}={v:.3f}' for j, v in zip(JOINT_NAMES, joint_positions))
        )
        ok = self._plan_and_execute(
            name,
            joint_positions=joint_positions,
        )
        if not ok:
            self.get_logger().error(f'[{name}] motion failed')
        else:
            self.get_logger().info(f'[{name}] complete')

    def move_to_base_position(self, x: float, y: float, z: float, roll: float = None, pitch: float = None, yaw: float = None, name: str = ''):
        '''Position and Rotation absolute in the BASE_LINK-Frame.'''
        # set log step for torque logging
        self.torque_logger.set_step(name)

        _, current_quat = self._get_ee_pose()
        target_quat = self._apply_rotation_absolut(current_quat, roll, pitch, yaw)

        self.get_logger().info(f'[{name}] Absolute position = ({x:.3f}, {y:.3f}, {z:.3f})')
        ok = self._plan_and_execute(
            name,
            position=[x, y, z],
            quat_xyzw=target_quat,
            cartesian=False,
        )
        if not ok:
            self.get_logger().error(f'[{name}] motion failed')
        else:
            self.get_logger().info(f'[{name}] complete')

    def move_to_relative_position(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, name: str = ''):
        '''Position and Rotation relative to the current EE pose.'''
        # set log step for torque logging
        self.torque_logger.set_step(name)

        current_pos, current_quat = self._get_ee_pose()
        target = [current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz]
        target_quat = self._apply_rotation_relativ(current_quat, roll, pitch, yaw)

        self.get_logger().info(
            f'[{name}] Relativ Δpos = ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  '
            f'Δrot = ({math.degrees(roll):+.1f}°,{math.degrees(pitch):+.1f}°,{math.degrees(yaw):+.1f}°)'
            f'Current-Pos = ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}), Current-Rot = ({math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[0]):.1f}°,{math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[1]):.1f}°,{math.degrees(Rotation.from_quat(current_quat).as_euler("xyz")[2]):.1f}°)'
        )
        ok = self._plan_and_execute(
            name,
            position=target,
            quat_xyzw=target_quat,
            cartesian=False,
        )
        if not ok:
            self.get_logger().error(f'[{name}] motion failed')
        else:
            self.get_logger().info(f'[{name}] complete')

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

        if not self._wait_for_joint_states():
            return 1
        if not self._wait_for_tf():
            return 1

        #self.move_to_joint_position(HOME_POSITION, 'home')
        #self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self.move_to_joint_position(PICK_JOINT_POSITION, 'pick')
        #self.move_to_relative_position(dz=-0.10, name='pick-down')

        #time.sleep(1) # vaccum on

        #self.move_to_relative_position(dz=0.10, name='pick-up')
        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        self.record_data_point('data_recording_center')

        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(0),
            pitch=math.radians(30),
            name='data_recording_front'
        )
        self.record_data_point('data_recording_front')

        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(-30),
            pitch=math.radians(0),
            name='data_recording_left'
        )
        self.record_data_point('data_recording_left')

        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(0),
            pitch=math.radians(-30),
            name='data_recording_right'
        )
        self.record_data_point('data_recording_right')

        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        self.move_to_relative_position(
            dx=0.0,
            dy=0.0,
            roll=math.radians(30),
            pitch=math.radians(0),
            name='data_recording_back'
        )
        self.record_data_point('data_recording_back')

        self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self.move_to_joint_position(START_JOINT_POSITION, 'start')
        #self.move_to_relative_position(dz=-0.10, name='place-down')

        #time.sleep(1) # vaccum off

        #self.move_to_relative_position(dz=0.10, name='place-up')
        #self.move_to_joint_position(HOME_POSITION, 'home')

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
        # Cancel any in-flight trajectory execution so the joint_trajectory_controller
        # returns to idle and the program can be rerun without restarting the server.
        # The executor must still be spinning while cancel_execution() sends its request.
        try:
            if node.moveit2.query_state() == MoveIt2State.EXECUTING:
                node.moveit2.cancel_execution()
                time.sleep(0.5)  # give the cancel request time to be processed
        except Exception:
            pass
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
