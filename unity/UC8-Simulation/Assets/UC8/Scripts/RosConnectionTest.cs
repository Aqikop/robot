using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RosConnectionTest : MonoBehaviour
{
    ROSConnection ros;
    float timer = 0f;
    int counter = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("/uc8/unity_to_ros");
        ros.Subscribe<StringMsg>("/uc8/ros_to_unity", OnRosMessage);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1f)
        {
            timer = 0f;
            counter++;
            StringMsg msg = new StringMsg("Hello from Unity #" + counter);
            ros.Publish("/uc8/unity_to_ros", msg);
        }
    }

    void OnRosMessage(StringMsg msg)
    {
        Debug.Log("Received from ROS: " + msg.data);
    }
}
