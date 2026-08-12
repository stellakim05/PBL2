using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RaspimousePublish : MonoBehaviour
{
    ROSConnection ros;
    public string cmdVelTopic = "cmd_vel";
    public string leftMotorTopic = "left_motor_speed";
    public string rightMotorTopic = "right_motor_speed";

    public float linearSpeed = 0.2f;
    public float angularSpeed = 1.0f;

    private float moveX, moveZ;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<Float32Msg>(leftMotorTopic);
        ros.RegisterPublisher<Float32Msg>(rightMotorTopic);
    }

    void Update()
    {
        moveX = Input.GetAxis("Vertical");
        moveZ = Input.GetAxis("Horizontal");

        if (moveX != 0 || moveZ != 0)
        {
            PublishCommand();
        }
    }

    void PublishCommand()
    {
        TwistMsg twist = new TwistMsg();

        twist.linear.x = moveX * linearSpeed;
        twist.angular.z = -moveZ * angularSpeed;  // Negative because Unity's left is positive, but ROS uses right-hand rule

        ros.Publish(cmdVelTopic, twist);

        // Calculate and publish motor speeds
        float leftSpeed = (float)((twist.linear.x - twist.angular.z * 0.09 / 2) / 0.2);
        float rightSpeed = (float)((twist.linear.x + twist.angular.z * 0.09 / 2) / 0.2);

        leftSpeed = Mathf.Clamp(leftSpeed, -1f, 1f);
        rightSpeed = Mathf.Clamp(rightSpeed, -1f, 1f);

        ros.Publish(leftMotorTopic, new Float32Msg(leftSpeed));
        ros.Publish(rightMotorTopic, new Float32Msg(rightSpeed));

        // Debug.Log($"Published: Linear X = {twist.linear.x}, Angular Z = {twist.angular.z}");
        // Debug.Log($"Motor speeds: Left = {leftSpeed}, Right = {rightSpeed}");
    }
}