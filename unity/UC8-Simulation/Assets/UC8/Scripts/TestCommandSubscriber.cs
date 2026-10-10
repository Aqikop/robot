using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// UC8 Week 3, Bai 3.3 (stage 1) - subscribes to a simple bool command
/// from ROS and toggles the test cube's color. This only proves the
/// ROS -> Unity direction works before we move to a real joint command.
/// Attach to an empty GameObject (alongside ROSConnection) and assign
/// Target Renderer to the test cube's Mesh Renderer in the Inspector.
/// </summary>
public class TestCommandSubscriber : MonoBehaviour
{
    [Header("Bridge config - keep in sync with docs/interfaces.md")]
    public string topicName = "/uc8/test_command";

    [Header("Visual feedback")]
    public Renderer targetRenderer;
    public Color onColor = Color.green;
    public Color offColor = Color.white;

    ROSConnection ros;
    bool currentState;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BoolMsg>(topicName, OnCommandReceived);

        if (targetRenderer == null)
        {
            Debug.LogWarning("[TestCommandSubscriber] No targetRenderer assigned - " +
                              "drag the Cube's Renderer into the Inspector field.");
        }
        else
        {
            targetRenderer.material.color = offColor;
        }
    }

    void OnCommandReceived(BoolMsg msg)
    {
        currentState = msg.data;
        Debug.Log($"[TestCommandSubscriber] Received command: {currentState}");

        if (targetRenderer != null)
        {
            targetRenderer.material.color = currentState ? onColor : offColor;
        }
    }
}
