import io
import json
import math
import sys
import threading
import time
import types
import zipfile
from pathlib import Path

import h5py
from lbr_fri_idl.msg import LBRState
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np

import tf2_ros
from pymoveit2 import MoveIt2, MoveIt2State
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

from robotik_projekt.pick_place_iiwa14_config import (
    BASE_LINK,
    DATA_RECORDING_SAMPLE_TIMEOUT,
    DATA_RECORDING_SETTLE_TIME,
    END_EFFECTOR,
    GROUP_NAME,
    JOINT_NAMES,
    MAX_ACCELERATION,
    MAX_VELOCITY,
    START_JOINT_POSITION,
    WAIT_TIMEOUT,
)

# -----------------------------------------------------------------------------
# Model artifacts
# -----------------------------------------------------------------------------
# Files produced by `Modelllernen.ipynb` after `model.save(...)` / `joblib.dump(...)`
# are expected to live next to `training_data_ready.csv` (i.e. in the workspace
# root from which the node is launched).
DEFAULT_MODEL_DIR = Path.cwd()
DEFAULT_MODEL_PATH = DEFAULT_MODEL_DIR / 'object_measurement_model.keras'
DEFAULT_SCALER_PATH = DEFAULT_MODEL_DIR / 'object_measurement_scaler.joblib'

# Order of the 35 input features expected by the trained model.
# Layout: 7 axes × 5 positions (center, front, left, right, back)
SAMPLE_POSITIONS = ('center', 'front', 'left', 'right', 'back')

# Mapping from `run_job()` step labels to the 5 sample positions.
# The order here MUST match the training CSV column layout, which the
# notebook produces in alphabetical order: center, front, left, right, back.
STEP_TO_POSITION = {
    'data_recording_center': 'center',
    'data_recording_front':  'front',
    'data_recording_left':   'left',
    'data_recording_right':  'right',
    'data_recording_back':   'back',
}


# -----------------------------------------------------------------------------
# Lightweight inference helpers
# -----------------------------------------------------------------------------

class StandardScalerLite:
    '''Small runtime replacement for sklearn.preprocessing.StandardScaler.'''

    def __init__(self, mean_, scale_, with_mean: bool = True, with_std: bool = True):
        self.mean_ = np.asarray(mean_, dtype=np.float32)
        self.scale_ = np.asarray(scale_, dtype=np.float32)
        self.with_mean = with_mean
        self.with_std = with_std

    def transform(self, x: np.ndarray) -> np.ndarray:
        result = np.asarray(x, dtype=np.float32)
        if self.with_mean:
            result = result - self.mean_
        if self.with_std:
            result = result / self.scale_
        return result


class _StandardScalerJoblibShim:
    '''Allows joblib to unpickle a StandardScaler when sklearn is unavailable.'''

    def __setstate__(self, state):
        self.__dict__.update(state)


class NumpyKerasModel:
    '''Runs the saved Sequential/Dense Keras model without importing TensorFlow.'''

    def __init__(self, layers: list[dict]):
        self.layers = layers

    @classmethod
    def load(cls, model_path: Path) -> 'NumpyKerasModel':
        with zipfile.ZipFile(model_path) as archive:
            config = json.loads(archive.read('config.json'))
            weights_data = archive.read('model.weights.h5')

        layers = []
        with h5py.File(io.BytesIO(weights_data), 'r') as weights_file:
            for layer_config in config['config']['layers']:
                if layer_config['class_name'] == 'InputLayer':
                    continue
                if layer_config['class_name'] != 'Dense':
                    raise ValueError(
                        f"Unsupported layer type: {layer_config['class_name']}"
                    )

                name = layer_config['config']['name']
                activation = layer_config['config'].get('activation', 'linear')
                use_bias = layer_config['config'].get('use_bias', True)
                group = weights_file[f'layers/{name}/vars']
                kernel = np.asarray(group['0'], dtype=np.float32)
                bias = (
                    np.asarray(group['1'], dtype=np.float32)
                    if use_bias else np.zeros(kernel.shape[1], dtype=np.float32)
                )
                layers.append({
                    'name': name,
                    'kernel': kernel,
                    'bias': bias,
                    'activation': activation,
                })

        return cls(layers)

    def predict(self, x: np.ndarray, verbose: int = 0) -> np.ndarray:
        del verbose
        output = np.asarray(x, dtype=np.float32)
        for layer in self.layers:
            output = output @ layer['kernel'] + layer['bias']
            activation = layer['activation']
            if activation == 'relu':
                output = np.maximum(output, 0.0)
            elif activation == 'linear':
                pass
            else:
                raise ValueError(
                    f"Unsupported activation in layer {layer['name']}: {activation}"
                )
        return output


def _install_sklearn_scaler_shim():
    sklearn_module = types.ModuleType('sklearn')
    preprocessing_module = types.ModuleType('sklearn.preprocessing')
    data_module = types.ModuleType('sklearn.preprocessing._data')
    data_module.StandardScaler = _StandardScalerJoblibShim
    sys.modules.setdefault('sklearn', sklearn_module)
    sys.modules.setdefault('sklearn.preprocessing', preprocessing_module)
    sys.modules.setdefault('sklearn.preprocessing._data', data_module)


def load_standard_scaler(scaler_path: Path) -> StandardScalerLite:
    from joblib import load as joblib_load

    try:
        scaler = joblib_load(str(scaler_path))
    except ModuleNotFoundError as exc:
        if not exc.name or not exc.name.startswith('sklearn'):
            raise
        _install_sklearn_scaler_shim()
        scaler = joblib_load(str(scaler_path))

    return StandardScalerLite(
        mean_=scaler.mean_,
        scale_=scaler.scale_,
        with_mean=getattr(scaler, 'with_mean', True),
        with_std=getattr(scaler, 'with_std', True),
    )


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class MeasureObject(Node):

    def __init__(self, model_path: Path, scaler_path: Path):
        super().__init__('measure_object_iiwa14')

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

        # Latest axis torques (updated by the LBRState callback).
        self._axis_torque = [0.0] * 7
        self._last_lbr_state_time = 0.0
        self._torque_lock = threading.Lock()
        self.data_recording_settle_time = DATA_RECORDING_SETTLE_TIME
        self.data_recording_sample_timeout = DATA_RECORDING_SAMPLE_TIMEOUT

        self.create_subscription(LBRState, '/lbr/lbr_state', self._on_lbr_state, 10)

        # Load model + scaler.
        self.get_logger().info(f'Loading NumPy inference model from {model_path}')
        self.model = NumpyKerasModel.load(model_path)
        self.get_logger().info(f'Loading scaler from {scaler_path}')
        self.scaler = load_standard_scaler(scaler_path)

    # -------------------------------------------------------------------------
    # Callbacks & Helper Methods
    # -------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState):
        if not self._joint_state_received:
            self.get_logger().info('Joint States available.')
        self._joint_state = msg
        self._joint_state_received = True

    def _on_lbr_state(self, msg: LBRState):
        with self._torque_lock:
            self._axis_torque = list(msg.external_torque)
            self._last_lbr_state_time = time.time()

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
        '''Sets angles to absolute values – missing axes from current orientation.'''
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

    def record_data_point(self, step: str):
        '''
        Records the current axis torques for `step` (one of the 5
        `data_recording_*` labels) into `self._samples[step]`.
        '''
        if self.data_recording_settle_time > 0.0:
            time.sleep(self.data_recording_settle_time)

        with self._torque_lock:
            axis_torque = list(self._axis_torque)

        position = STEP_TO_POSITION.get(step)
        if position is None:
            self.get_logger().error(f'Unknown data-recording step: {step}')
            return

        self._samples[position] = axis_torque
        self.get_logger().info(
            f'[{step}/{position}] recorded axis torques: '
            + ', '.join(f'A{i+1}={v:.4f}' for i, v in enumerate(axis_torque))
        )

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
        '''Moves to a joint position (values in radians).'''
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
    # Prediction
    # -------------------------------------------------------------------------

    def _build_feature_vector(self) -> np.ndarray:
        '''
        Build the (1, 35) input vector for the model in the exact column
        order used during training:
            A1_center, A1_front, A1_left, A1_right, A1_back,
            A2_center, ..., A7_back
        '''
        row = []
        for axis in range(7):
            for pos in SAMPLE_POSITIONS:
                row.append(self._samples[pos][axis])
        return np.array(row, dtype=np.float32).reshape(1, -1)

    def predict(self) -> tuple[float, float, float]:
        '''Runs the trained model on the recorded samples.'''
        x_input = self._build_feature_vector()
        self.get_logger().info(f'Feature vector: {x_input.tolist()}')
        x_scaled = self.scaler.transform(x_input)
        prediction = self.model.predict(x_scaled, verbose=0)[0]
        x, y, z = float(prediction[0]), float(prediction[1]), float(prediction[2])
        return x, y, z

    # -------------------------------------------------------------------------
    # Main process
    # -------------------------------------------------------------------------

    def run_job(self) -> int:
        self.get_logger().info('=== Measure Object launched ===')

        if not self._wait_for_joint_states():
            return 1
        if not self._wait_for_tf():
            return 1

        # Per-position samples (axis torques) recorded during the motion.
        self._samples: dict[str, list[float]] = {p: [0.0] * 7 for p in SAMPLE_POSITIONS}

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

        # ------------------------------------------------------------------
        # Predict x, y, z from the recorded torques and print the result.
        # ------------------------------------------------------------------
        self.get_logger().info('Running model inference...')
        x_pred, y_pred, z_pred = self.predict()

        self.get_logger().info('=== Measurement result ===')
        self.get_logger().info(f'  x = {x_pred:.4f}')
        self.get_logger().info(f'  y = {y_pred:.4f}')
        self.get_logger().info(f'  z = {z_pred:.4f}')
        print('\n=== Measurement result ===')
        print(f'  x = {x_pred:.4f}')
        print(f'  y = {y_pred:.4f}')
        print(f'  z = {z_pred:.4f}')

        return 0


# -----------------------------------------------------------------------------
# Entry Point
# -----------------------------------------------------------------------------

def main() -> int:
    rclpy.init()
    node = MeasureObject(
        model_path=DEFAULT_MODEL_PATH,
        scaler_path=DEFAULT_SCALER_PATH,
    )

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
