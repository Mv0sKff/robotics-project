import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def make_pose(
    frame_id: str,
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class PickPlaceServoNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_servo")

        callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "lbr_A1",
                "lbr_A2",
                "lbr_A3",
                "lbr_A4",
                "lbr_A5",
                "lbr_A6",
                "lbr_A7",
            ],
            base_link_name="world",
            end_effector_name="lbr_link_ee",
            group_name="arm",
            callback_group=callback_group,
        )

        # Direkter Publisher auf den echten Servo-Topic
        self._servo_pub = self.create_publisher(
            TwistStamped,
            "/lbr/servo_node/delta_twist_cmds",
            10,
        )

        self.home_pose = make_pose("world", -0.40, 0.00, 0.90)
        self.over_pick_place_pose = make_pose("world", -0.45, 0.00, 0.75)

    def move_to_pose(self, name: str, pose: PoseStamped) -> None:
        pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"MoveIt pose goal -> {name}")

        self.moveit2.move_to_pose(
            position=pose.pose.position,
            quat_xyzw=[
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            cartesian=False,
        )
        self.moveit2.wait_until_executed()

    def servo_for(
        self,
        duration: float,
        linear=(0.0, 0.0, 0.0),
        angular=(0.0, 0.0, 0.0),
        rate_hz: float = 20.0,
        label: str = "",
    ) -> None:
        if label:
            self.get_logger().info(f"Servo -> {label}")

        msg = TwistStamped()
        msg.header.frame_id = "world"

        period = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear[0])
            msg.twist.linear.y = float(linear[1])
            msg.twist.linear.z = float(linear[2])
            msg.twist.angular.x = float(angular[0])
            msg.twist.angular.y = float(angular[1])
            msg.twist.angular.z = float(angular[2])

            self._servo_pub.publish(msg)
            time.sleep(period)

        # Stop-Kommando mehrfach senden
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = "world"
        for _ in range(5):
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            self._servo_pub.publish(stop_msg)
            time.sleep(period)

    def run_sequence(self) -> None:
        self.move_to_pose("home", self.home_pose)
        self.move_to_pose("over_pick_place", self.over_pick_place_pose)

        self.servo_for(
            duration=2.0,
            linear=(0.0, 0.0, -0.05),
            label="down to pick/place",
        )

        self.servo_for(
            duration=2.0,
            linear=(0.0, 0.0, 0.05),
            label="up from pick/place",
        )

        self.servo_for(
            duration=2.5,
            linear=(0.0, 0.05, 0.0),
            label="swing left",
        )

        self.servo_for(
            duration=5.0,
            linear=(0.0, -0.05, 0.0),
            label="swing right",
        )

        self.servo_for(
            duration=2.5,
            linear=(0.0, 0.05, 0.0),
            label="back to center",
        )

        self.servo_for(
            duration=2.0,
            linear=(0.0, 0.0, -0.05),
            label="down again",
        )

        self.servo_for(
            duration=2.0,
            linear=(0.0, 0.0, 0.05),
            label="up again",
        )

        self.move_to_pose("over_pick_place", self.over_pick_place_pose)
        self.move_to_pose("home", self.home_pose)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = PickPlaceServoNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.run_sequence()
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()