# Week 04 Report (DRAFT) — Static Pick-and-Place (R4)

**Date:** 2026-10-10 (in progress)
**Author:** Nguyễn Mạnh Khôi (10423062). Roles are not formally assigned; the team is working on all parts.
**Scope:** Bài 4.1–4.5. This draft covers only the closed-loop execution foundation (Step 1). The pick scene, poses, state machine and 10 trials are **not done yet** and must be added before submission.

---

## 1. Status

| Item | Status | Evidence |
|---|---|---|
| Unity executes trajectories and publishes actual joint states | Done | `UR10eTrajectoryExecutor.cs`, test log below |
| ROS action server reports done only after measured joints settle | Done | `uc8_trajectory_bridge.py`, `experiments/week04/bridge_log.csv` |
| MoveIt Plan + Execute drives Unity through that server | Done (one pose tested) | RViz run, bridge `SUCCEEDED` |
| Scene: cube, table, bin, tool frame (Bài 4.1) | Not started | |
| Grasp / release and attached collision object (Bài 4.3) | Not started | |
| Pick/place poses (Bài 4.2) | Not started | |
| State machine with timeout and FAILED (Bài 4.4) | Not started | |
| 10-trial table incl. forced grasp failure (Bài 4.5) | Not started | |

"Done" means tested with evidence. Nothing in the not-started rows should be reported as progress.

## 2. What was built

- **Unity side:** receives `/uc8/joint_trajectory`, maps joints by name, interpolates between trajectory points, drives the articulation joints, and publishes measured joints on `/joint_states` at 50 Hz. `/uc8/trajectory_cancel` makes it hold its current pose.
- **ROS side:** `FollowJointTrajectory` action server `unity_arm_controller/follow_joint_trajectory`. SUCCEEDED requires the measured joints within 0.035 rad of the final point and below 0.02 rad/s for 0.2 s. It rejects goals whose start is more than 0.1 rad from the measured state, and aborts and requests a hold if joint states go stale or the arm does not settle within the planned duration plus 3 s.
- **MoveIt wiring:** `launch/uc8_moveit_unity.launch` plus `config/uc8_controllers.yaml` (no fake controller; `allowed_start_tolerance` 0.05).

## 3. Results

Tests run through the action server directly (no MoveIt):

| Test | Result | Total time (planned 3.0 s) |
|---|---|---|
| Elbow +0.3 rad | SUCCEEDED, max error 0.0144 rad | 3.71 s |
| Elbow −0.3 rad | SUCCEEDED | 3.59 s |
| Shoulder lift +0.2 rad | SUCCEEDED | 3.71 s |
| Elbow +0.3 rad with start state 0.5 rad off | Rejected: start state mismatch 0.500 rad > 0.100 | 0.00 s |

MoveIt run (RViz Plan + Execute, group `manipulator`, "up" pose): SUCCEEDED, planned 2.53 s, elapsed 4.56 s, max error 0.0000 rad. The upright pose has almost no gravity load, so this error is not representative of reaching poses.

Raw rows are in `experiments/week04/bridge_log.csv`.

## 4. Problems found

1. **First test aborted** with error code −5, 0.1796 rad (about 10°) short of the goal after 6.01 s. The tolerance check refused to report done, as designed. The fix was disabling the leftover Week 3 test objects in the scene (consistent with those scripts competing with the executor for the joint drives); the same test then passed. The bridge script was also updated at that point, but only to report the worst joint in the abort message, not to change behavior.
2. **MoveIt reported 0 controllers** (`CONTROL_FAILED`). MoveIt checks for the action server only at startup. Fixed by starting the bridge before MoveIt (documented in `docs/setup.md`).
3. **Joint-state warning** `not newer than the previous state`. The warning is gone when the system is started in the documented order (endpoint, Play, bridge, MoveIt). Duplicate millisecond timestamps were a suspected cause, so the executor now stamps strictly increasing, but the warning's disappearance was observed after fixing the start order, so the cause was not isolated.
4. **Large velocities at Play start.** Start-up transient; the arm calms within seconds. Wait before sending trajectories.

## 5. Not yet tested

- Stopping Play mid-move should abort as stale (hold requested). Not run yet.
- MoveIt execution on an extended (reaching) pose, where gravity droop will show.

## 6. Limitations carried forward

- Steady-state joint droop under gravity; to be measured at the tool on reaching poses.
- About 0.7 to 2 s of settle time beyond the planned duration; affects cycle time in the 10-trial table.
- Order-dependent start-up (bridge before MoveIt).
- Simulation only.

## 7. Next actions

1. Run the two pending tests above.
2. Pick scene in Unity: cube, table, bin, tool frame, attach/release, reset, `/uc8/cube_pose`.
3. Pose config, attached collision object in MoveIt, state machine with timeouts, 10 trials with failures included.

---

## Weekly report summary (draft, update before submitting)

> This week we committed to static pick-and-place (R4). So far we completed the closed-loop execution foundation: Unity executes trajectories and publishes measured joints, and a ROS action server reports completion only after the arm has arrived and settled, verified directly and through MoveIt Plan and Execute. One test initially failed (10° short) and was corrected; we also found and documented a MoveIt start-order dependency. The pick scene, grasp logic, state machine and 10-trial table are not started yet. Next action: build the pick scene, then poses and state machine. Evidence is in `experiments/week04/bridge_log.csv` and the commit on GitHub.
