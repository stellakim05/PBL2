using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class simplearm_subscriber : MonoBehaviour
{
    public ArticulationBody baseArticulationBody;
    public ArticulationBody armArticulationBody;
    public float baseLinearSpeed = 1.0f;
    public float baseAngularSpeed = 1.0f;
    public float armSpeed = 1.0f;
    public float armMinAngle = -180f; // Adjust these values for wider motion
    public float armMaxAngle = 180f;  // Adjust these values for wider motion

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", MoveBase);
        ros.Subscribe<Float64Msg>("arm_joint_position_controller/command", MoveArm);
        Debug.Log("SimpleArm_Subscriber started and subscribed to topics.");
    }

    void MoveBase(TwistMsg twist)
    {
        if (baseArticulationBody != null)
        {
            // Corrected mapping of ROS coordinates to Unity coordinates
            Vector3 linearVelocity = new Vector3((float)twist.linear.x, 0, (float)twist.linear.y) * baseLinearSpeed;
            Vector3 angularVelocity = new Vector3(0, (float)twist.angular.z * baseAngularSpeed, 0);

            baseArticulationBody.velocity = baseArticulationBody.transform.TransformDirection(linearVelocity);
            baseArticulationBody.angularVelocity = angularVelocity;

            Debug.Log($"Applied base velocity: {linearVelocity}, Angular velocity: {angularVelocity}");
        }
        else
        {
            Debug.LogError("BaseArticulationBody is null!");
        }
    }

    void MoveArm(Float64Msg armPosition)
    {
        if (armArticulationBody != null)
        {
            float targetAngle = Mathf.Clamp((float)armPosition.data * Mathf.Rad2Deg, armMinAngle, armMaxAngle);
            var drive = armArticulationBody.xDrive;
            drive.target = targetAngle;
            drive.stiffness = 10000; // Adjust this value as needed
            drive.damping = 100;     // Adjust this value as needed
            armArticulationBody.xDrive = drive;

            Debug.Log($"Applied arm rotation target: {targetAngle}");
        }
        else
        {
            Debug.LogError("ArmArticulationBody is null!");
        }
    }

    void OnDrawGizmos()
    {
        if (baseArticulationBody != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(baseArticulationBody.transform.position, baseArticulationBody.velocity);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(baseArticulationBody.transform.position, baseArticulationBody.angularVelocity);
        }
    }
}