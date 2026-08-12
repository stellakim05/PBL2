using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class MyPublisher : MonoBehaviour
{
    public GameObject simpleBox;
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    public float moveSpeed = 1.0f;
    public float turnSpeed = 1.0f;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        TwistMsg twist = new TwistMsg();

        // Forward/Backward movement
        if (Input.GetKey(KeyCode.W))
        {
            twist.linear.x = moveSpeed;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            twist.linear.x = -moveSpeed;
        }

        // Left/Right turning
        if (Input.GetKey(KeyCode.A))
        {
            twist.angular.z = turnSpeed;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            twist.angular.z = -turnSpeed;
        }

        // Publish the message to ROS
        ros.Publish(topicName, twist);
    }
}
