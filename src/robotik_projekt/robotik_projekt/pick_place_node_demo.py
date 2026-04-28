import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class PickPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_node")

        # Initialize the action client for controlling the robot's joints
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for action server to become available...")

        self.get_logger().info("Action server available.")

    def move_to_joint_positions(self, positions: list[float], duration: int = 5) -> None:
        if len(positions) != 7:
            self.get_logger().error("Invalid number of joint positions.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance.sec = 1

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 7
        point.time_from_start.sec = duration

        goal.trajectory.joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]
        goal.trajectory.points.append(point)

        self.get_logger().info(f"Moving to position: {positions}")
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Movement goal was not accepted.")
            return

        self.get_logger().info("Movement goal accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2)

        result = result_future.result()
        if result is None:
            self.get_logger().error("No response received from action server.")
            return

        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error("Movement failed.")
            return

        self.get_logger().info("Movement completed successfully.")


def main(args=None) -> None:
    rclpy.init(args=args)

    node = PickPlaceNode()

    duration = 8

    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pick_position = [0.3, 0.6, 0.0, -1.0, 0.0, 1.0, 0.0]
    place_position = [-0.3, 0.6, 0.0, -1.0, 0.0, 1.0, 0.0]

    node.get_logger().info("Fahre zur Home-Position.")
    node.move_to_joint_positions(home_position, duration=duration)

    node.get_logger().info("Fahre zur Pick-Position.")
    node.move_to_joint_positions(pick_position, duration=duration)

    node.get_logger().info("Fahre zur Place-Position.")
    node.move_to_joint_positions(place_position, duration=duration)

    node.get_logger().info("Fahre zurück zur Home-Position.")
    node.move_to_joint_positions(home_position, duration=duration)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()