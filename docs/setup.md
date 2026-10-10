# UC8 Setup & Run Guide

Simulation only (no real robot). Robot: UR10e. Stack: Ubuntu 20.04 · ROS 1 Noetic · MoveIt 1 · Unity 2022.3.62f3.
Exact versions and commit hashes: see `docs/versions.md`.

## Repository layout

| Path | Contents |
|---|---|
| `ros_ws/src/uc8_robot_sorting/` | Our ROS package: `scripts/`, `launch/`, `config/` |
| `unity/UC8-Simulation/` | Unity project (Assets, Packages, ProjectSettings) |
| `experiments/weekXX/` | Raw logs and results per week |
| `docs/` | Setup, versions, weekly reports |

`ROS-TCP-Endpoint` is **not** in this repo. It is an external dependency (see below).

## First-time setup

```bash
git clone https://github.com/Aqikop/robot.git ~/Downloads/UC8-Robotic-Sorting
cd ~/Downloads/UC8-Robotic-Sorting/ros_ws/src

# External dependency, pinned to the commit recorded in docs/versions.md
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
git -C ROS-TCP-Endpoint checkout <commit-from-versions.md>

cd ..
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin_make
source devel/setup.bash
chmod +x src/uc8_robot_sorting/scripts/*.py
rospack find uc8_robot_sorting        # must print a path
```

Tip: add `source ~/Downloads/UC8-Robotic-Sorting/ros_ws/devel/setup.bash` to `~/.bashrc`
so every new terminal knows the workspace.

Unity: open `unity/UC8-Simulation` in Unity Hub with Editor **2022.3.62f3**. Packages restore
from `Packages/manifest.json`. Do not let Unity auto-update packages.

## Unity scene checklist (scene `Week02_UR10e_Control`)

- `ROSConnection`: protocol **ROS1**, IP `127.0.0.1`, port `10000`, connect on startup ON.
- On `ur10e_robot`: `UR10eTrajectoryExecutor` **enabled**.
- **Disabled**: `UR10eJointStateSubscriber`, `Controller`, `TestSingleJointCommand`,
  `TestCommandSubscriber` (they also write joint drives and fight the executor).
- Save the scene in Edit mode (Ctrl+S). Changes made in Play mode are discarded.

## Run order (MoveIt executing through Unity)

Open a new terminal for each step and source `/opt/ros/noetic/setup.bash` and
`ros_ws/devel/setup.bash` in each.

1. Endpoint (also starts roscore):
   `roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000`
2. Press **Play** in Unity. Wait until the arm has stopped moving (it flails briefly at start).
3. Bridge (action server):
   `rosrun uc8_robot_sorting uc8_trajectory_bridge.py _log_csv:=$HOME/bridge_log.csv`
   Wait for: `uc8_trajectory_bridge ready on /unity_arm_controller/follow_joint_trajectory`
4. MoveIt + RViz:
   `roslaunch uc8_robot_sorting uc8_moveit_unity.launch`
   Look for `Added FollowJointTrajectory controller for unity_arm_controller`.
5. In RViz (planning group `manipulator`, start state `current`): **Plan**, then **Execute**.

**Order matters.** MoveIt looks for the action server only at startup. If the bridge restarts,
restart step 4 too. Do not run `demo.launch`: its fake controller also publishes `/joint_states`.

## Quick test without MoveIt

With steps 1–3 running:

```bash
rostopic echo -n 1 /joint_states                                   # real values from Unity
rosrun uc8_robot_sorting send_test_trajectory.py --delta 0.3       # expect SUCCEEDED
rosrun uc8_robot_sorting send_test_trajectory.py --delta 0.3 --stale-start   # expect rejection
```

## Troubleshooting (problems we actually hit)

| Symptom | Cause / fix |
|---|---|
| `package 'uc8_robot_sorting' not found` | Terminal did not source the workspace: `source ros_ws/devel/setup.bash` |
| `chmod: cannot access .../scripts/*.py` | `scripts/` folder missing: `mkdir scripts`, copy the `.py` files in, `chmod +x`, `catkin_make` |
| `rosrun ... uc8_trajectory_bridge.py --help` seems frozen | It is a long-running node, not a CLI tool. Run it normally with the endpoint up |
| MoveIt: `Returned 0 controllers in list`, `CONTROL_FAILED` | Bridge was not running when MoveIt started. Start the bridge first, then restart MoveIt |
| Bridge `ABORTED ... did not settle ... err X rad on <joint>` | Check the worst joint named in the message. Other scripts driving joints? Drive damping / force limit? See limitations |
| Bridge `start state mismatch` | Arm moved since planning. Wait until calm and plan again from `current` |
| `New joint state ... is not newer than the previous state` (WARN) | Seen with the wrong start order; gone when following the run order above. The executor also stamps strictly increasing. If a steady stream persists, investigate |
| Huge joint velocities in the first `/joint_states` sample | Start-up transient. Wait for the arm to calm before sending trajectories |

## Known limitations

- Simulation only. Simulation safety is not a certification of real-robot safety.
- Joint drive has steady-state droop under gravity; measured max error is recorded per run in `bridge_log.csv`.
- Moves take about 0.6 to 2 s longer than planned because of the settle check (counts toward cycle time).
- Unity reconnection to the endpoint is silent (no confirmation log).
