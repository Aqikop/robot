#!/usr/bin/env python3
"""
Send one small test move to the Unity-backed action server (no MoveIt needed).

  rosrun uc8_robot_sorting send_test_trajectory.py --joint elbow_joint --delta 0.3 --duration 3.0
  rosrun uc8_robot_sorting send_test_trajectory.py --delta 0.3 --stale-start   # negative test: must be rejected
"""
import argparse
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

if __name__ == "__main__":
    rospy.init_node("uc8_send_test_trajectory")
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="elbow_joint", choices=JOINTS)
    ap.add_argument("--delta", type=float, default=0.3, help="rad to add to the joint")
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--ns", default="unity_arm_controller/follow_joint_trajectory")
    ap.add_argument("--stale-start", action="store_true",
                    help="start the trajectory 0.5 rad away from the real state (should be rejected)")
    args = ap.parse_args(rospy.myargv()[1:])

    js = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
    cur = dict(zip(js.name, js.position))
    start = [cur[n] for n in JOINTS]
    goal = list(start)
    goal[JOINTS.index(args.joint)] += args.delta
    if args.stale_start:
        start[JOINTS.index(args.joint)] += 0.5

    client = actionlib.SimpleActionClient(args.ns, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        raise SystemExit("action server not found - is uc8_trajectory_bridge.py running?")

    g = FollowJointTrajectoryGoal()
    g.trajectory.joint_names = JOINTS
    p0 = JointTrajectoryPoint(positions=start, time_from_start=rospy.Duration(0.0))
    p1 = JointTrajectoryPoint(positions=goal, time_from_start=rospy.Duration(args.duration))
    g.trajectory.points = [p0, p1]

    t0 = time.monotonic()
    client.send_goal(g)
    client.wait_for_result()
    res = client.get_result()
    print("state=%s  error_code=%s  msg='%s'  total=%.2fs (planned %.1fs)" % (
        actionlib.GoalStatus.to_string(client.get_state()) if hasattr(actionlib.GoalStatus, "to_string")
        else client.get_state(),
        res.error_code, res.error_string, time.monotonic() - t0, args.duration))
