using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UR10eJointStateSubscriber : MonoBehaviour
{
    public string topic = "/joint_states";
    public float stiffness = 10000f;
    public float damping = 100f;
    public float forceLimit = 1000f;

    private readonly string[] rosNames = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    private readonly string[] linkNames = {
        "shoulder_link", "upper_arm_link", "forearm_link",
        "wrist_1_link", "wrist_2_link", "wrist_3_link"
    };
    private readonly Dictionary<string, int> index =
        new Dictionary<string, int>();
    private ArticulationBody[] joints;
    private float[] targets;
    private bool received, warned;

    private void Start()
    {
        joints = new ArticulationBody[6];
        targets = new float[6];
        var bodies = GetComponentsInChildren<ArticulationBody>(true);
        for (int i = 0; i < 6; i++)
        {
            foreach (var body in bodies)
            {
                if (body.name != linkNames[i]) continue;
                if (joints[i] != null)
                {
                    Debug.LogError("Duplicate link: " + linkNames[i]);
                    enabled = false; return;
                }
                joints[i] = body;
            }
            if (joints[i] == null || joints[i].jointType !=
                ArticulationJointType.RevoluteJoint)
            {
                Debug.LogError("Missing revolute link: " + linkNames[i]);
                enabled = false; return;
            }
            index.Add(rosNames[i], i);
            var drive = joints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.targetVelocity = 0f;
            joints[i].xDrive = drive;
        }
        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointStateMsg>(topic, Receive);
        Debug.Log("UR10e: waiting for " + topic);
    }
    private void Receive(JointStateMsg message)
    {
        if (!isActiveAndEnabled || message.name == null ||
            message.position == null) return;
        var next = new float[6];
        var found = new bool[6];
        int count = Math.Min(message.name.Length,
            message.position.Length);
        for (int n = 0; n < count; n++)
        {
            if (!index.TryGetValue(message.name[n], out int i))
                continue;
            double angle = message.position[n];
            if (double.IsNaN(angle) || double.IsInfinity(angle))
                continue;
            next[i] = (float)(angle * Mathf.Rad2Deg);
            found[i] = true;
        }
        for (int i = 0; i < 6; i++)
        {
            if (found[i]) continue;
            if (!warned)
            {
                Debug.LogWarning("Missing ROS joints. Received: " +
                    string.Join(", ", message.name));
                warned = true;
            }
            return;
        }
        targets = next;
        if (!received)
            Debug.Log("UR10e: receiving all 6 ROS joints.");
        received = true;
    }

    private void FixedUpdate()
    {
        if (!received) return;
        for (int i = 0; i < 6; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = targets[i];
            joints[i].xDrive = drive;
        }
    }
}
