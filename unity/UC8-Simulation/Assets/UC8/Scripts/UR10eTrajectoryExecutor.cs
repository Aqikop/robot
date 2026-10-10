using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;

/// <summary>
/// Week 4 / Step 1 - closed-loop execution on the Unity side.
///
///  ROS  -> Unity : /uc8/joint_trajectory  (trajectory_msgs/JointTrajectory)
///  ROS  -> Unity : /uc8/trajectory_cancel (std_msgs/Empty)  -> hold current pose
///  Unity -> ROS  : /joint_states          (sensor_msgs/JointState, ACTUAL measured joints)
///
/// This script REPLACES UR10eJointStateSubscriber (and any keyboard Controller) on ur10e_robot.
/// Disable those before pressing Play, otherwise two scripts fight over xDrive.target.
///
/// Playback finishing is NOT "arrived". The ROS action server decides Done from the
/// measured joint states this script publishes.
/// </summary>
public class UR10eTrajectoryExecutor : MonoBehaviour
{
    // Fixed ROS joint order of the UR10e (base -> tip). Messages are mapped to slots BY NAME.
    static readonly string[] RosJointNames =
    {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    [Header("Joints - slot order must match RosJointNames (base -> tip)")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    [Header("Topics")]
    public string trajectoryTopic = "/uc8/joint_trajectory";
    public string cancelTopic = "/uc8/trajectory_cancel";
    public string jointStateTopic = "/joint_states";
    public float jointStateRateHz = 50f;

    [Header("Drive (Week 3 used stiffness 10000)")]
    public float stiffness = 10000f;
    public float damping = 100f;
    public float forceLimit = 1000f;
    [Tooltip("Experiment: turn gravity off on the arm links to remove steady-state droop. Compare droop with this on/off and log it.")]
    public bool disableLinkGravity = false;

    [Header("Debug")]
    public bool logEvents = true;

    const long UnixEpochTicks = 621355968000000000L;
    long lastStampNs;

    ROSConnection ros;
    bool initialised;
    double nextPublishTime;

    // Active trajectory (already reordered into slot order, radians)
    bool executing;
    double[][] posRad;
    double[] tFrom;
    double tStart;
    int cursor;
    bool pendingStart;

    readonly double[] holdRad = new double[6];

    void Start()
    {
        if (!JointsValid()) AutoFillJoints();
        if (!JointsValid())
        {
            Debug.LogError("[Executor] Need 6 revolute ArticulationBodies. Fill 'joints' by hand.");
            enabled = false;
            return;
        }

        foreach (var j in joints)
        {
            var d = j.xDrive;
            d.stiffness = stiffness;
            d.damping = damping;
            d.forceLimit = forceLimit;
            j.xDrive = d;
            if (disableLinkGravity) j.useGravity = false;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(jointStateTopic);
        ros.Subscribe<JointTrajectoryMsg>(trajectoryTopic, OnTrajectory);
        ros.Subscribe<EmptyMsg>(cancelTopic, OnCancel);
    }

    bool JointsValid()
    {
        if (joints == null || joints.Length != 6) return false;
        foreach (var j in joints) if (j == null) return false;
        return true;
    }

    [ContextMenu("Auto-fill joints from children")]
    void AutoFillJoints()
    {
        var found = new List<ArticulationBody>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            if (ab.jointType == ArticulationJointType.RevoluteJoint) found.Add(ab);

        if (found.Count != 6)
        {
            Debug.LogError($"[Executor] Expected 6 revolute joints, found {found.Count}.");
            return;
        }
        joints = found.ToArray();
        for (int i = 0; i < 6; i++)
            Debug.Log($"[Executor] slot {i} ({RosJointNames[i]}) <- GameObject '{joints[i].name}'  (check this order!)");
    }

    [ContextMenu("Print joint unit check (Play mode)")]
    void PrintUnitCheck()
    {
        for (int i = 0; i < 6; i++)
            Debug.Log($"[Executor] {RosJointNames[i]}: jointPosition[0]={joints[i].jointPosition[0]:F4}  " +
                      $"xDrive.target={joints[i].xDrive.target:F3} deg ({joints[i].xDrive.target * Mathf.Deg2Rad:F4} rad)");
    }

    // ---------------- ROS callbacks ----------------

    void OnTrajectory(JointTrajectoryMsg msg)
    {
        if (!initialised) { Debug.LogWarning("[Executor] Trajectory ignored: not initialised yet."); return; }
        if (msg.points == null || msg.points.Length == 0) { Debug.LogError("[Executor] Empty trajectory rejected."); return; }

        int n = msg.joint_names.Length;
        var slotOf = new int[n];
        var seen = new bool[6];
        for (int k = 0; k < n; k++)
        {
            slotOf[k] = Array.IndexOf(RosJointNames, msg.joint_names[k]);
            if (slotOf[k] < 0) { Debug.LogError($"[Executor] Unknown joint '{msg.joint_names[k]}' - trajectory rejected."); return; }
            seen[slotOf[k]] = true;
        }
        foreach (bool s in seen)
            if (!s) { Debug.LogError("[Executor] Trajectory must contain all 6 joints - rejected."); return; }

        int p = msg.points.Length;
        var newPos = new double[p][];
        var newT = new double[p];
        for (int i = 0; i < p; i++)
        {
            var pt = msg.points[i];
            if (pt.positions == null || pt.positions.Length != n) { Debug.LogError("[Executor] Bad point size - rejected."); return; }
            newPos[i] = new double[6];
            for (int k = 0; k < n; k++) newPos[i][slotOf[k]] = pt.positions[k];
            newT[i] = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
            if (i > 0 && newT[i] < newT[i - 1]) { Debug.LogError("[Executor] time_from_start not increasing - rejected."); return; }
        }

        posRad = newPos;
        tFrom = newT;
        cursor = 0;
        pendingStart = true; // t0 is taken inside FixedUpdate
        executing = true;
        if (logEvents) Debug.Log($"[Executor] Trajectory accepted: {p} points, {newT[p - 1]:F2} s");
    }

    void OnCancel(EmptyMsg _)
    {
        if (!initialised) return;
        executing = false;
        pendingStart = false;
        CaptureHold();
        ApplyTargets(holdRad);
        if (logEvents) Debug.Log("[Executor] Cancel received - holding current pose.");
    }

    // ---------------- Physics loop ----------------

    void FixedUpdate()
    {
        if (!initialised)
        {
            CaptureHold();          // joints are only readable after the first physics step
            ApplyTargets(holdRad);
            initialised = true;
            nextPublishTime = Time.fixedTimeAsDouble;
            return;
        }

        if (executing)
        {
            if (pendingStart) { tStart = Time.fixedTimeAsDouble; pendingStart = false; }
            double t = Time.fixedTimeAsDouble - tStart;
            var cmd = Sample(t, out bool finished);
            ApplyTargets(cmd);
            if (finished)
            {
                executing = false;
                Array.Copy(cmd, holdRad, 6);   // keep holding the final point
                if (logEvents) Debug.Log("[Executor] Playback finished (NOT proof of arrival - ROS checks measured joints).");
            }
        }

        if (Time.fixedTimeAsDouble >= nextPublishTime)
        {
            PublishJointStates();
            nextPublishTime += 1.0 / jointStateRateHz;
            if (nextPublishTime < Time.fixedTimeAsDouble) nextPublishTime = Time.fixedTimeAsDouble;
        }
    }

    double[] Sample(double t, out bool finished)
    {
        int last = tFrom.Length - 1;
        finished = false;
        if (t <= tFrom[0]) return posRad[0];
        if (t >= tFrom[last]) { finished = true; return posRad[last]; }

        while (cursor < last - 1 && t >= tFrom[cursor + 1]) cursor++;
        double span = tFrom[cursor + 1] - tFrom[cursor];
        double a = span > 1e-9 ? (t - tFrom[cursor]) / span : 1.0;
        var res = new double[6];
        for (int i = 0; i < 6; i++)
            res[i] = posRad[cursor][i] + a * (posRad[cursor + 1][i] - posRad[cursor][i]);
        return res;
    }

    void CaptureHold()
    {
        for (int i = 0; i < 6; i++) holdRad[i] = joints[i].jointPosition[0]; // radians for revolute (verify with the unit-check menu)
    }

    void ApplyTargets(double[] rad)
    {
        for (int i = 0; i < 6; i++)
        {
            var d = joints[i].xDrive;
            d.target = (float)(rad[i] * Mathf.Rad2Deg);
            joints[i].xDrive = d;
        }
    }

    void PublishJointStates()
    {
        var pos = new double[6];
        var vel = new double[6];
        for (int i = 0; i < 6; i++)
        {
            pos[i] = joints[i].jointPosition[0];
            vel[i] = joints[i].jointVelocity[0];
        }

        // 100 ns resolution + strictly increasing: back-to-back FixedUpdates must never share a stamp.
        long ns = (DateTime.UtcNow.Ticks - UnixEpochTicks) * 100L;
        if (ns <= lastStampNs) ns = lastStampNs + 1000L;
        lastStampNs = ns;
        var msg = new JointStateMsg
        {
            header = new HeaderMsg
            {
                frame_id = "",
                stamp = new TimeMsg { sec = (uint)(ns / 1000000000L), nanosec = (uint)(ns % 1000000000L) }
            },
            name = (string[])RosJointNames.Clone(),
            position = pos,
            velocity = vel,
            effort = new double[0]
        };
        ros.Publish(jointStateTopic, msg);
    }
}
