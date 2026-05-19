import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


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
            f'Digital-Output ch={channel} -> {"ON" if state else "OFF"}'
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
