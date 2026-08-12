using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;

public class RaspimouseController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    public GameObject raspimouse;
    public float movementSpeed = 1.0f;
    public float rotationSpeed = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, MoveRobot);

        // Debug line to log the URI and port
        Debug.Log("ROS Connection URI: " + ros.RosIPAddress + ":" + ros.RosPort);
    }

    void MoveRobot(TwistMsg cmdVel)
    {
        Vector3 linear = cmdVel.linear.From<FLU>();
        Vector3 angular = cmdVel.angular.From<FLU>();

        raspimouse.transform.Translate(linear * movementSpeed * Time.deltaTime);
        raspimouse.transform.Rotate(angular * rotationSpeed * Time.deltaTime);
    }
}
