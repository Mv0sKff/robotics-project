#!/usr/bin/env python3
"""Starter script for pose-based motion planning with MoveIt 2 in Python.

This is a template for the KUKA LBR stack.
It assumes that the MoveIt bringup is already running and that moveit_py is installed.

Important TODOs before real use:
- verify the planning group name (default: "arm")
- verify the namespace / robot_name (default: "lbr")
- verify the base frame (for example "world" or the robot base link)
- verify the end-effector link name (for example the flange / TCP link)
- verify all target poses in RViz before moving real hardware
"""

import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


@dataclass
class PoseGoal:
    name: str
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


def make_pose_stamped(frame_id: str, goal: PoseGoal) -> PoseStamped:
    """Build a PoseStamped message from a PoseGoal."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = goal.x
    msg.pose.position.y = goal.y
    msg.pose.position.z = goal.z
    msg.pose.orientation.x = goal.qx
    msg.pose.orientation.y = goal.qy
    msg.pose.orientation.z = goal.qz
    msg.pose.orientation.w = goal.qw
    return msg


def plan_and_execute(robot: MoveItPy, planning_component, logger) -> bool:
    """Plan and execute the currently configured goal."""
    logger.info("Planning trajectory ...")
    plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed.")
        return False

    logger.info("Executing plan ...")
    robot.execute(plan_result.trajectory, controllers=[])
    return True


def move_to_pose(robot: MoveItPy, planning_component, logger, frame_id: str, pose_link: str, goal: PoseGoal) -> bool:
    """Plan from current state to one pose goal and execute it."""
    logger.info(f"Setting goal pose: {goal.name}")
    planning_component.set_start_state_to_current_state()
    planning_component.set_goal_state(
        pose_stamped_msg=make_pose_stamped(frame_id, goal),
        pose_link=pose_link,
    )
    return plan_and_execute(robot, planning_component, logger)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    logger = rclpy.logging.get_logger("robotik_moveit_pose_demo")

    # ===== TODO: adapt these values to your real setup =====
    robot_name = "lbr"              # namespace / robot name used by your stack
    planning_group = "arm"          # arm planning group from the MoveIt config
    base_frame = "world"            # TODO: maybe this should be the robot base link instead
    pose_link = "tool0"             # TODO: replace with your actual TCP / flange / EE link

    # Create the MoveItPy interface and get the planning component.
    # Some setups only need MoveItPy(node_name="...").
    # The explicit robot_name argument is kept here because your stack uses /lbr.
    try:
        robot = MoveItPy(node_name="robotik_moveit_pose_demo")
    except TypeError:
        logger.error(
            "Could not create MoveItPy with this API on your system. "
            "Check whether moveit_py is installed and available in your ROS distro."
        )
        rclpy.shutdown()
        return 1

    planning_component = robot.get_planning_component(planning_group)
    logger.info("MoveItPy interface created.")

    # ===== Example pose sequence =====
    # Replace these numbers with the real measured target poses for your setup.
    # The sequence mirrors your intended motion:
    # home -> over_pick_place -> pick_place -> over_pick_place ->
    # swing_left -> over_pick_place -> swing_right -> over_pick_place ->
    # pick_place -> over_pick_place -> home
    goals = [
        PoseGoal("home",            -0.40,  0.00, 0.90),
        PoseGoal("over_pick_place", -0.40,  0.00, 0.70),
        PoseGoal("pick_place",      -0.40,  0.00, 0.55),
        PoseGoal("over_pick_place", -0.40,  0.00, 0.70),
        PoseGoal("swing_left",      -0.30,  0.20, 0.70),
        PoseGoal("over_pick_place", -0.40,  0.00, 0.70),
        PoseGoal("swing_right",     -0.30, -0.20, 0.70),
        PoseGoal("over_pick_place", -0.40,  0.00, 0.70),
        PoseGoal("pick_place",      -0.40,  0.00, 0.55),
        PoseGoal("over_pick_place", -0.40,  0.00, 0.70),
        PoseGoal("home",            -0.40,  0.00, 0.90),
    ]

    for goal in goals:
        ok = move_to_pose(robot, planning_component, logger, base_frame, pose_link, goal)
        if not ok:
            logger.error(f"Sequence aborted at goal: {goal.name}")
            rclpy.shutdown()
            return 2

        # Small pause so the log output stays readable.
        time.sleep(0.2)

    logger.info("Sequence completed.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
