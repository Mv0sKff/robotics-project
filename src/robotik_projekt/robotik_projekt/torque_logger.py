import csv
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from lbr_fri_idl.msg import LBRState
from rclpy.node import Node


class TorqueLogger:
    def __init__(
        self,
        node: Node,
        data_recording_settle_time: float,
        data_recording_sample_timeout: float,
    ):
        self._node = node
        self._current_step = 'init'
        self._axis_torque = [0.00] * 7
        self._last_lbr_state_time = 0.0
        self._lock = threading.Lock()
        self.data_recording_settle_time = data_recording_settle_time
        self.data_recording_sample_timeout = data_recording_sample_timeout

        log_dir = Path.cwd() / 'log' / 'torque_logs'
        Path.mkdir(log_dir, parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_file_path = log_dir / f'torque_{timestamp}.csv'
        self._data_recording_csv_file_path = log_dir / f'data_recording_{timestamp}.csv'

        with open(self._csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'step'] + [f'A_{i+1}' for i in range(7)])
        self._node.get_logger().info(f'Touque-Log: {self._csv_file_path}')

        with open(self._data_recording_csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['timestamp', 'step'] + [f'A_{i+1}' for i in range(7)]
            )
        self._node.get_logger().info(
            f'Data-recording torque log: {self._data_recording_csv_file_path}'
        )

    def set_step(self, step: str):
        self._current_step = step

    def on_lbr_state(self, msg: LBRState):
        axis_torque = list(msg.external_torque)
        with self._lock:
            self._axis_torque = axis_torque
            self._last_lbr_state_time = time.time()
        self._log_torque(self._current_step, axis_torque)

    def record_data_point(self, step: str) -> bool:
        if self.data_recording_settle_time > 0.0:
            time.sleep(self.data_recording_settle_time)

        request_time = time.time()
        deadline = request_time + self.data_recording_sample_timeout
        axis_torque = None

        while rclpy.ok() and time.time() < deadline:
            with self._lock:
                if self._last_lbr_state_time >= request_time:
                    axis_torque = list(self._axis_torque)
                    break
            time.sleep(0.01)

        if axis_torque is None:
            self._node.get_logger().error(f'No current LBRState sample received for {step}')
            return False

        row = [f'{time.time():.3f}', step] + [f'{value:.4f}' for value in axis_torque]
        with open(self._data_recording_csv_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow(row)

        self._node.get_logger().info(f'[{step}] current data-recording values saved')
        return True

    def _log_torque(self, step: str, axis_torque: list[float]):
        row = [f'{time.time():.3f}', step] + [f'{t:.4f}' for t in axis_torque]
        with open(self._csv_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow(row)
