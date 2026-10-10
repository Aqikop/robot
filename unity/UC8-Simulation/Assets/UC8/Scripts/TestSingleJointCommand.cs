using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// UC8 Week 3 - Bai 3.3 (command) + Bai 3.4 (command vs feedback).
///
/// - Subscribes to a target angle (radians) for ONE joint and drives it.
/// - Publishes the joint's ACTUAL measured angle back to ROS continuously.
/// - Only declares "Done" once the measured angle is within threshold of
///   the commanded angle AND the joint has settled (low velocity) - not
///   just because a command message was received.
///
/// IMPORTANT: disable UR10eJointStateSubscriber on ur10e_robot while testing
/// this script - otherwise /joint_states feedback will fight this command
/// every FixedUpdate. Re-enable it when done.
///
/// Attach to an empty GameObject (alongside ROSConnection) and assign
/// Target Joint to the ArticulationBody of the joint you're testing
/// (e.g. forearm_link's ArticulationBody, which drives elbow_joint).
/// </summary>
public class TestSingleJointCommand : MonoBehaviour
{
    [Header("Bridge config - keep in sync with docs/interfaces.md")]
    public string commandTopic = "/uc8/test_joint_command";
    public string feedbackTopic = "/uc8/test_joint_feedback";
    public string doneTopic = "/uc8/test_joint_done";

    [Header("Which joint we're driving (for logging only)")]
    public string jointNameLabel = "elbow_joint";

    [Header("What we're driving")]
    public ArticulationBody targetJoint;

    [Header("Safety clamp (degrees) - keep small while testing")]
    public float minDegrees = -30f;
    public float maxDegrees = 30f;

    [Header("Done criteria")]
    // 2.0 deg, not 1.0: measured steady-state droop under gravity at
    // stiffness=10000 was ~1.46 deg on elbow_joint (Bai 3.4 test, 2026-10-04).
    // Revisit if stiffness/damping changes, or per-joint if droop differs.
    public float angleThresholdDeg = 2.0f;      // how close counts as "arrived"
    public float velocityThresholdDegPerSec = 1.0f; // how still counts as "settled"
    public float feedbackRateHz = 10f;

    ROSConnection ros;
    float commandedDeg;
    bool isDone = true; // starts true: joint begins at rest at its current pose
    float timeSinceLastFeedback;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(commandTopic, OnJointCommandReceived);
        ros.RegisterPublisher<Float64Msg>(feedbackTopic);
        ros.RegisterPublisher<BoolMsg>(doneTopic);

        if (targetJoint == null)
        {
            Debug.LogWarning("[TestSingleJointCommand] No targetJoint assigned - " +
                              "drag the joint's ArticulationBody into the Inspector field.");
        }
        else
        {
            commandedDeg = CurrentAngleDeg();
        }
    }

    void OnJointCommandReceived(Float64Msg msg)
    {
        if (targetJoint == null) return;

        float radians = (float)msg.data;
        float degrees = radians * Mathf.Rad2Deg;
        float clamped = Mathf.Clamp(degrees, minDegrees, maxDegrees);

        if (!Mathf.Approximately(clamped, degrees))
        {
            Debug.LogWarning($"[TestSingleJointCommand] {jointNameLabel}: requested {degrees:F2} deg, " +
                              $"clamped to {clamped:F2} deg (safety limit while testing).");
        }

        ArticulationDrive drive = targetJoint.xDrive;
        drive.target = clamped;
        targetJoint.xDrive = drive;

        commandedDeg = clamped;
        isDone = false; // a new command means we are no longer "arrived" until feedback says so

        Debug.Log($"[TestSingleJointCommand] {jointNameLabel}: commanded {radians:F4} rad " +
                  $"({clamped:F2} deg)");
    }

    void Update()
    {
        if (targetJoint == null) return;

        timeSinceLastFeedback += Time.deltaTime;
        if (timeSinceLastFeedback >= 1f / feedbackRateHz)
        {
            timeSinceLastFeedback = 0f;
            PublishFeedback();
        }
    }

    void PublishFeedback()
    {
        float currentDeg = CurrentAngleDeg();
        float currentDegPerSec = CurrentVelocityDegPerSec();

        // Publish the measured angle, in radians, same units as the command topic.
        Float64Msg feedbackMsg = new Float64Msg(currentDeg * Mathf.Deg2Rad);
        ros.Publish(feedbackTopic, feedbackMsg);

        bool withinAngle = Mathf.Abs(currentDeg - commandedDeg) < angleThresholdDeg;
        bool settled = Mathf.Abs(currentDegPerSec) < velocityThresholdDegPerSec;
        bool nowDone = withinAngle && settled;

        if (nowDone != isDone)
        {
            isDone = nowDone;
            ros.Publish(doneTopic, new BoolMsg(isDone));

            if (isDone)
            {
                Debug.Log($"[TestSingleJointCommand] {jointNameLabel}: DONE - " +
                          $"measured {currentDeg:F2} deg vs commanded {commandedDeg:F2} deg, " +
                          $"velocity {currentDegPerSec:F2} deg/s");
            }
            else
            {
                Debug.Log($"[TestSingleJointCommand] {jointNameLabel}: moving - " +
                          $"measured {currentDeg:F2} deg, target {commandedDeg:F2} deg");
            }
        }
    }

    float CurrentAngleDeg()
    {
        // jointPosition[0] is the single revolute DOF, in radians.
        return targetJoint.jointPosition.dofCount > 0
            ? targetJoint.jointPosition[0] * Mathf.Rad2Deg
            : 0f;
    }

    float CurrentVelocityDegPerSec()
    {
        return targetJoint.jointVelocity.dofCount > 0
            ? targetJoint.jointVelocity[0] * Mathf.Rad2Deg
            : 0f;
    }
}
