using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class MySubscriber : MonoBehaviour
{
    public GameObject simpleBox;
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    private Rigidbody rb;
    public float moveSpeed = 1.0f;
    public float turnSpeed = 1.0f;

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        rb = simpleBox.GetComponent<Rigidbody>();
        
        // Register the subscriber to the topic
        ros.Subscribe<TwistMsg>(topicName, ReceiveTwist);
    }

    void ReceiveTwist(TwistMsg twist)
    {
        // Apply linear velocity
        Vector3 movement = new Vector3((float)twist.linear.x, 0, (float)twist.linear.z);
        rb.velocity = movement * moveSpeed;

        // Apply angular velocity for rotation
        float turn = (float)twist.angular.z;
        Vector3 rotation = Vector3.up * turn * turnSpeed * Time.deltaTime;
        Quaternion deltaRotation = Quaternion.Euler(rotation);
        rb.MoveRotation(rb.rotation * deltaRotation);
    }
}
