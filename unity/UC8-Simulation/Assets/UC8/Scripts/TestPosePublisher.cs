using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

/// <summary>
/// UC8 Week 3 - publishes a test object's pose to ROS so we can verify
/// the Unity <-> ROS bridge agrees on frame, units, and axis signs.
/// Attach this to an empty GameObject (alongside your ROSConnection object)
/// and assign Cube Transform in the Inspector.
/// </summary>
public class TestPosePublisher : MonoBehaviour
{
    [Header("Bridge config - keep in sync with docs/interfaces.md")]
    public string topicName = "/uc8/test_pose";
    public string frameId = "world";
    public float publishRateHz = 5f;

    [Header("What we're tracking")]
    public Transform cubeTransform;

    ROSConnection ros;
    float timeSinceLastPublish;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);

        if (cubeTransform == null)
        {
            Debug.LogWarning("[TestPosePublisher] No cubeTransform assigned - " +
                              "drag your test cube into the Inspector field.");
        }
    }

    void Update()
    {
        if (cubeTransform == null) return;

        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish >= 1f / publishRateHz)
        {
            PublishPose();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishPose()
    {
        // Stamp left at default (zero) for this axis-check test - we only need
        // frame_id here. Real timestamps come into play later (W11 latency work).
        HeaderMsg header = new HeaderMsg();
        header.frame_id = frameId;

        // Unity is left-handed, Y-up. ROS (REP-103) is right-handed, Z-up.
        // Standard Unity->ROS conversion: ros(x,y,z) = (unity.z, -unity.x, unity.y)
        Vector3 u = cubeTransform.position;
        PointMsg rosPosition = new PointMsg(u.z, -u.x, u.y);

        // Identity quaternion as a safe, valid default - never leave this all-zero.
        QuaternionMsg rosOrientation = new QuaternionMsg(0, 0, 0, 1);

        PoseStampedMsg poseMsg = new PoseStampedMsg(header, new PoseMsg(rosPosition, rosOrientation));

        ros.Publish(topicName, poseMsg);
    }
}
