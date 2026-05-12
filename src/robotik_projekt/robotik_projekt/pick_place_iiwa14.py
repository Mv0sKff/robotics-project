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
# Konfiguration
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
WAIT_TIMEOUT     = 10.0   # Sekunden fuer Joint-State- und TF-Wartezeit

# -----------------------------------------------------------------------------
# Digital Output Client
# -----------------------------------------------------------------------------

class DigitalOutputClient:
    '''Allgemeiner Client zum Schalten digitaler Ausgänge.'''

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
        self._states: dict[int, bool] = {ch: False for ch in range(num_channels)}
        self._clients = {
            ch: self._node.create_client(
                SetBool,
                self._service_name_template.format(channel=ch + 1),
                callback_group=callback_group,
            )
            for ch in range(num_channels)
        }

    def wait_for_services(self, channel: int, state: bool, timeout_sec: float = 5.0) -> bool:
        '''Wartet, bis der digitale Ausgang den gewünschten Zustand hat.'''
        start_time = time.time()

        while not self.get_output(channel) == state:
            if time.time() - start_time > timeout_sec:
                self._node.get_logger().error(
                    f'Timeout beim Warten auf Digital-Output ch={channel} state={"EIN" if state else "AUS"}'
                )
                return False
            time.sleep(0.1)
        return True

    def set_output(self, channel: int, state: bool) -> bool:
        '''Schaltet einen digitalen Ausgang.'''
        if channel not in self._states:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} existiert nicht '
            )
            return False

        self._states[channel] = state
        self._node.get_logger().info(
            f'Digital-Output ch={channel} → {"EIN" if state else "AUS"}'
        )
        return self._call_service(channel, state)
        #return True

    def get_output(self, channel: int) -> bool | None:
        '''Gibt den zuletzt gesetzten Zustand eines Kanals zurück.'''
        return self._states.get(channel)

    def _call_service(self, channel: int, state: bool, timeout_sec: float = 2.0) -> bool:
        client = self._clients.get(channel)
        if client is None:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service-Client fehlt'
            )
            return False

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service nicht verfuegbar'
            )
            return False

        req = SetBool.Request()
        req.data = state
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service-Timeout'
            )
            return False

        resp = future.result()
        if resp is None:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} Service-Fehler'
            )
            return False

        if not resp.success:
            self._node.get_logger().error(
                f'Digital-Output ch={channel} nicht geschaltet: {resp.message}'
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
        self.digital_out = DigitalOutputClient(
            node=self,
            callback_group=self.callback_group,
            num_channels=4,
        )

        # Log-CSV-Datei
        self._current_step = 'init'
        self._axis_torque = [0.00] * 7
        self.create_subscription(LBRState, '/lbr/lbr_state', self._on_lbr_state, 10)

        log_dir  = Path.cwd() / 'log' / 'torque_logs'
        Path.mkdir(log_dir, parents=True, exist_ok=True)
        filename = datetime.now().strftime('torque_%Y%m%d_%H%M%S.csv')
        self._csv_file_path = log_dir / filename

        with open(self._csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'step'] + [f'A_{i+1}' for i in range(7)])
        self.get_logger().info(f'Touque-Log: {self._csv_file_path}')

    # -------------------------------------------------------------------------
    # Callbacks & Hilfsmethoden
    # -------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        if not self._joint_state_received:
            self.get_logger().info('Joint States verfügbar.')
        self._joint_state_received = True

    def _wait_for(self, condition_fn, label: str, timeout: float = WAIT_TIMEOUT) -> bool:
        '''Wartet, bis condition_fn() True zurückgibt oder Timeout eintritt.'''
        self.get_logger().info(f'Warte auf: {label}')
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
                self.get_logger().info('TF verfügbar.')
                return True
            except Exception:
                return False

        return self._wait_for(tf_available, f'TF {BASE_LINK} → {END_EFFECTOR}')

    def _get_ee_pose(self) -> tuple[list[float], list[float]]:
        '''
        Liest aktuelle Endeffektor-Pose relativ zu BASE_LINK.

        Rückgabe:
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
        '''Setzt Winkel absolut – fehlende Achsen aus aktueller Orientierung.'''
        if not any(v is not None for v in (roll, pitch, yaw)):
            return current_quat
        current_rpy = Rotation.from_quat(current_quat).as_euler('xyz')
        r = roll  if roll  is not None else current_rpy[0]
        p = pitch if pitch is not None else current_rpy[1]
        y = yaw   if yaw   is not None else current_rpy[2]
        return Rotation.from_euler('xyz', [r, p, y]).as_quat().tolist()

    def _apply_rotation_relativ(self, current_quat: list[float], roll: float = 0.0, pitch: float = 0.0,  yaw: float = 0.0) -> list[float]:
        '''Addiert Winkel auf aktuelle Orientierung drauf.'''
        current_rpy = Rotation.from_quat(current_quat).as_euler('xyz')
        r = current_rpy[0] + roll
        p = current_rpy[1] + pitch
        y = current_rpy[2] + yaw
        return Rotation.from_euler('xyz', [r, p, y]).as_quat().tolist()

    def _on_lbr_state(self, msg: LBRState):
        self._axis_torque = list(msg.external_torque)
        self._log_torque(self._current_step)

    def _log_torque(self, step: str):
        row = [f'{time.time():.3f}', step] + [f'{t:.4f}' for t in self._axis_torque]
        with open(self._csv_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow(row)

    # -------------------------------------------------------------------------
    # Bewegungsbefehle
    # -------------------------------------------------------------------------

    def move_to_joint_position(self, joint_positions: list[float], name: str = ''):
        '''Fährt eine Gelenkstellung an (Werte in Radiant).'''
        self.get_logger().info(
            f'[{name}] Gelenkposition = '
            + ', '.join(f'{j}={v:.3f}' for j, v in zip(JOINT_NAMES, joint_positions))
        )
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'[{name}] ✓ fertig')

    def move_to_base_position(self, x: float, y: float, z: float, roll: float = None, pitch: float = None, yaw: float = None, name: str = ''):
        '''Position und Rotation absolut im BASE_LINK-Frame.'''
        _, current_quat = self._get_ee_pose()
        target_quat = self._apply_rotation_absolut(current_quat, roll, pitch, yaw)

        self.get_logger().info(f'[{name}] Absolutepositon = ({x:.3f}, {y:.3f}, {z:.3f})')
        self.moveit2.move_to_pose(position=[x, y, z], quat_xyzw=target_quat, cartesian=False)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'[{name}] ✓ fertig')

    def move_to_relative_position(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, name: str = ''):
        '''Position und Rotation relativ zur aktuellen EE-Pose.h'''
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
        self.get_logger().info(f'[{name}] ✓ fertig')

    # -------------------------------------------------------------------------
    # Hauptablauf
    # -------------------------------------------------------------------------

    def run_job(self) -> int:
        self.get_logger().info('=== Pick & Place gestartet ===')

        #if not self._wait_for_joint_states():
        #    return 1
        #if not self._wait_for_tf():
        #    return 1
        self._current_step = 'home'
        self.move_to_joint_position(HOME_POSITION, 'Home')
        self._current_step = 'Start'
        self.move_to_joint_position(START_JOINT_POSITION, 'Start')
        self._current_step = 'Pick'
        self.move_to_joint_position(PICK_JOINT_POSITION, 'Pick')
        self._current_step = 'Pick-Down'
        self.move_to_relative_position(dz=-0.10, name='Pick-Down')
        #vaccum(true)
        self.digital_out.set_output(channel=3, state=True)
        #
        self._current_step = 'Pick-Up'
        self.move_to_relative_position(dz=0.10, name='Pick-Up')
        self._current_step = 'Start'
        self.move_to_joint_position(START_JOINT_POSITION, 'Start')
        #self._current_step = 'test0'
        #self.move_to_relative_position(
        #    dx=0.0,
        #    dy=0.0,
        #    roll=math.radians(30),
        #    pitch=math.radians(0),
        #    name='test0'
        #)
        #self.move_to_joint_position(START_JOINT_POSITION, 'Start')
        for _ in range(1):
            self._current_step = 'test1'
            self.move_to_relative_position(
                dx=0.0,
                dy=0.0,
                roll=math.radians(0),
                pitch=math.radians(30),
                name='test1'
            )
            self.move_to_joint_position(START_JOINT_POSITION, 'Start')
            self._current_step = 'test2'
            self.move_to_relative_position(
                dx=0.0,
                dy=0.0,
                roll=math.radians(-30),
                pitch=math.radians(0),
                name='test2'
            )
            self.move_to_joint_position(START_JOINT_POSITION, 'Start')
            self._current_step = 'test3'
            self.move_to_relative_position(
                dx=0.0,
                dy=0.0,
                roll=math.radians(0),
                pitch=math.radians(-30),
                name='test3'
            )
            self.move_to_joint_position(START_JOINT_POSITION, 'Start')
            self._current_step = 'test4'
            self.move_to_relative_position(
                dx=0.0,
                dy=0.0,
                roll=math.radians(30),
                pitch=math.radians(0),
                name='test4'
            )
            self.move_to_joint_position(START_JOINT_POSITION, 'Start')
        self._current_step = 'Start'
        self.move_to_joint_position(START_JOINT_POSITION, 'Start')
        self._current_step = 'Place-Down'
        self.move_to_relative_position(dz=-0.10, name='Place-Down')
        #vaccum(false)
        self.digital_out.set_output(channel=3, state=False)
        #
        self._current_step = 'Place-Up'
        self.move_to_relative_position(dz=0.10, name='Place-Up')
        self._current_step = 'Home'
        self.move_to_joint_position(HOME_POSITION, 'Home')

        self.get_logger().info('=== Pick & Place erfolgreich beendet ===')
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
        node.get_logger().warn('Abgebrochen.')
        exit_code = 130
    except Exception as exc:
        node.get_logger().error(f'Fehler: {exc}')
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
