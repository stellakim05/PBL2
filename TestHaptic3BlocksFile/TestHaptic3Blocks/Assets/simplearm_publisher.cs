using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class simplearm_publisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "arm_position";
    public string cmdVelTopicName = "cmd_vel_feedback";
    public float publishMessageFrequency = 0.5f;
    public GameObject armObject;
    public GameObject baseObject;

    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64Msg>(topicName);
        ros.RegisterPublisher<TwistMsg>(cmdVelTopicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Publish arm position
            Float64Msg armPositionMsg = new Float64Msg(armObject.transform.localRotation.eulerAngles.y);
            ros.Publish(topicName, armPositionMsg);

            // Publish base velocity (assuming you have a way to get current velocity)
            TwistMsg twistMsg = new TwistMsg(
                new Vector3Msg(baseObject.GetComponent<Rigidbody>().velocity.x, 0, baseObject.GetComponent<Rigidbody>().velocity.z),
                new Vector3Msg(0, baseObject.GetComponent<Rigidbody>().angularVelocity.y, 0)
            );
            ros.Publish(cmdVelTopicName, twistMsg);

            timeElapsed = 0;
        }
    }
}