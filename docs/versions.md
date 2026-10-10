# Versions

Record exact versions here. Do not write "latest". Entries marked **FILL** need the command
shown to be run once on the working machine and the output pasted in.

## Platform

| Component | Version | Source |
|---|---|---|
| OS | Ubuntu 20.04 LTS (focal) | `lsb_release -a` |
| ROS | ROS 1 Noetic (rosversion 1.17.4) | `rosversion -d`, `rosversion ros` |
| Unity Editor | 2022.3.62f3 | `unity/UC8-Simulation/ProjectSettings/ProjectVersion.txt` |
| Python | 3.x (system, Ubuntu 20.04) | `python3 --version` |

## ROS packages

| Package | Version | Command to fill |
|---|---|---|
| ros-noetic-moveit | 1.1.16-1focal.20250521.021831 | `dpkg-query -W -f='${Package}\t${Version}\n' ros-noetic-moveit` |
| ros-noetic-universal-robots | 1.5.0-1focal.20250521.021952 | `dpkg-query -W -f='${Package}\t${Version}\n' ros-noetic-universal-robots` |
| ur10e_moveit_config | from ros-noetic-universal-robots | `rospack find ur10e_moveit_config` |

## External repository (not vendored in this repo)

```
ROS-TCP-Endpoint
  repo:   https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
  branch: main
  commit: 993d366b8900bf9f3d2da444fde64c0379b4dc7c
```

## Unity packages

Pinned by `unity/UC8-Simulation/Packages/manifest.json` and `packages-lock.json` (committed).

| Package | Version / pin |
|---|---|
| ROS-TCP-Connector | Git URL, **not pinned to a tag**: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`. Resolved commit: `c27f00c6cf750d2d0564349b3039d19aa3925e7c` |
| URDF-Importer | v0.5.2, resolved commit `90f353e4352aae4df52fa2c05e49b804631d2a63`: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer#v0.5.2` |

The resolved commits come from the `"hash"` lines in `unity/UC8-Simulation/Packages/packages-lock.json`
(`grep -A8 'robotics' unity/UC8-Simulation/Packages/packages-lock.json`). The lock file is committed,
so everyone who opens the project resolves the same revisions.

## Our code

| What | Reference |
|---|---|
| Repo | https://github.com/Aqikop/robot |
| Week 4 step 1 commit | `2b5ff84` (Week 4 step 1: Unity trajectory executor, ROS action server bridge, MoveIt via Unity) |

Update this file whenever a package or Editor version changes, and tag release candidates
(`m1-demo`, `m2-demo`, `m3-demo`, `final-demo`) after recording their versions here.
