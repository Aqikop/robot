#!/usr/bin/env python3
"""
Week 4 / Step 1 - FollowJointTrajectory action server backed by Unity.

Flow:
  MoveIt (or the test client) --goal--> this node --/uc8/joint_trajectory--> Unity
  Unity --/joint_states (ACTUAL joints)--> this node --result--> caller

SUCCEEDED is reported only when the measured joints are within `goal_tolerance`
of the final point AND have been slower than `settle_velocity` for `settle_time`.
"Command sent" or "playback finished" never counts as done.

Aborts (hold requested via /uc8/trajectory_cancel) when:
  - joint states go stale during execution
  - the arm has not settled within trajectory duration + `timeout_margin`
Rejects goals whose first point is far from the measured state (plan from a stale start).
"""
import csv
import os
import time

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryFeedback,
                              FollowJointTrajectoryResult)
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class UnityTrajectoryBridge:
    def __init__(self):
        self.action_ns = rospy.get_param("~action_ns", "unity_arm_controller/follow_joint_trajectory")
        # Week 3 measured ~1.5 deg steady-state droop; 0.035 rad = 2.0 deg (same as the Week 3 done threshold).
        self.goal_tol = rospy.get_param("~goal_tolerance", 0.035)      # rad
        self.settle_vel = rospy.get_param("~settle_velocity", 0.02)    # rad/s
        self.settle_time = rospy.get_param("~settle_time", 0.2)        # s
        self.margin = rospy.get_param("~timeout_margin", 3.0)          # s after trajectory end
        self.max_state_age = rospy.get_param("~max_state_age", 0.5)    # s
        self.start_tol = rospy.get_param("~start_tolerance", 0.10)     # rad
        self.log_csv = rospy.get_param("~log_csv", "")                 # optional evidence log

        self.latest = None        # (wall_time, {name: (pos, vel)})
        self.traj_pub = rospy.Publisher("/uc8/joint_trajectory", JointTrajectory, queue_size=1)
        self.cancel_pub = rospy.Publisher("/uc8/trajectory_cancel", Empty, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)

        self.server = actionlib.SimpleActionServer(
            self.action_ns, FollowJointTrajectoryAction,
            execute_cb=self._execute, auto_start=False)
        self.server.start()
        rospy.loginfo("uc8_trajectory_bridge ready on /%s (goal_tol=%.3f rad)", self.action_ns, self.goal_tol)

    # ---------- state ----------
    def _on_joint_states(self, msg):
        vel = list(msg.velocity) if len(msg.velocity) == len(msg.name) else [0.0] * len(msg.name)
        state = {n: (p, v) for n, p, v in zip(msg.name, msg.position, vel)}
        self.latest = (time.monotonic(), state)  # receipt time, avoids ROS/Unity time-base issues

    def _fresh_state(self):
        if self.latest is None:
            return None
        age = time.monotonic() - self.latest[0]
        return self.latest[1] if age <= self.max_state_age else None

    def _hold(self):
        self.cancel_pub.publish(Empty())

    def _finish(self, kind, code, text, t0, planned, max_err):
        res = FollowJointTrajectoryResult()
        res.error_code = code
        res.error_string = text
        elapsed = time.monotonic() - t0 if t0 else 0.0
        rospy.loginfo("[bridge] %s: %s (elapsed %.2fs, planned %.2fs, max_err %.4f rad)",
                      kind, text, elapsed, planned, max_err)
        self._log_row(kind, planned, elapsed, max_err, text)
        if kind == "SUCCEEDED":
            self.server.set_succeeded(res)
        elif kind == "PREEMPTED":
            self.server.set_preempted(res)
        else:
            self.server.set_aborted(res)

    def _log_row(self, kind, planned, elapsed, max_err, text):
        if not self.log_csv:
            return
        new = not os.path.exists(self.log_csv)
        with open(self.log_csv, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["wall_time", "result", "planned_s", "elapsed_s", "max_err_rad", "note"])
            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), kind, "%.3f" % planned,
                        "%.3f" % elapsed, "%.5f" % max_err, text])

    # ---------- action ----------
    def _execute(self, goal):
        traj = goal.trajectory
        R = FollowJointTrajectoryResult
        if not traj.points:
            return self._finish("ABORTED", R.INVALID_GOAL, "empty trajectory", None, 0.0, 0.0)
        if set(traj.joint_names) != set(JOINTS):
            return self._finish("ABORTED", R.INVALID_JOINTS,
                                "joint names must be the 6 UR10e joints, got %s" % list(traj.joint_names),
                                None, 0.0, 0.0)

        state = self._fresh_state()
        if state is None:
            return self._finish("ABORTED", R.INVALID_GOAL,
                                "no fresh /joint_states from Unity (is Play mode on?)", None, 0.0, 0.0)

        final = {n: p for n, p in zip(traj.joint_names, traj.points[-1].positions)}
        first = {n: p for n, p in zip(traj.joint_names, traj.points[0].positions)}
        start_err = max(abs(first[n] - state[n][0]) for n in JOINTS)
        if start_err > self.start_tol:
            return self._finish("ABORTED", R.INVALID_GOAL,
                                "start state mismatch %.3f rad > %.3f (re-plan from the real state)"
                                % (start_err, self.start_tol), None, 0.0, start_err)

        planned = traj.points[-1].time_from_start.to_sec()
        self.traj_pub.publish(traj)
        t0 = time.monotonic()
        settled_since = None
        max_err = float("inf")
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self._hold()
                return self._finish("PREEMPTED", R.SUCCESSFUL, "preempted by new goal/cancel", t0, planned, max_err)

            state = self._fresh_state()
            if state is None:
                self._hold()
                return self._finish("ABORTED", R.INVALID_GOAL, "joint states went stale - hold requested",
                                    t0, planned, max_err)

            elapsed = time.monotonic() - t0
            errs = [abs(final[n] - state[n][0]) for n in JOINTS]
            vels = [abs(state[n][1]) for n in JOINTS]
            max_err = max(errs)

            fb = FollowJointTrajectoryFeedback()
            fb.joint_names = JOINTS
            fb.actual.positions = [state[n][0] for n in JOINTS]
            self.server.publish_feedback(fb)

            if elapsed >= planned:
                if max_err <= self.goal_tol and max(vels) <= self.settle_vel:
                    if settled_since is None:
                        settled_since = time.monotonic()
                    if time.monotonic() - settled_since >= self.settle_time:
                        return self._finish("SUCCEEDED", R.SUCCESSFUL, "arrived and settled", t0, planned, max_err)
                else:
                    settled_since = None
                if elapsed > planned + self.margin:
                    self._hold()
                    worst = JOINTS[errs.index(max_err)]
                    rospy.logwarn("[bridge] per-joint error (rad): %s",
                                  ", ".join("%s=%.3f" % (n.replace("_joint", ""), e) for n, e in zip(JOINTS, errs)))
                    return self._finish("ABORTED", R.GOAL_TOLERANCE_VIOLATED,
                                        "did not settle within tolerance (err %.4f rad on %s)" % (max_err, worst),
                                        t0, planned, max_err)
            rate.sleep()

        self._hold()
        self._finish("ABORTED", R.INVALID_GOAL, "node shutdown", t0, planned, max_err)


if __name__ == "__main__":
    rospy.init_node("uc8_trajectory_bridge")
    UnityTrajectoryBridge()
    rospy.spin()
