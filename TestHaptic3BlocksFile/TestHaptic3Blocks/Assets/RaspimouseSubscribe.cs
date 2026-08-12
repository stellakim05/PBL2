using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RaspimouseSubscribe : MonoBehaviour
{
    ROSConnection ros;
    public string cmdVelTopic = "cmd_vel";

    public ArticulationBody baseLink;
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    public float maxLinearSpeed = 0.2f;
    public float maxAngularSpeed = 1.0f;
    public float wheelRadius = 0.024f;
    public float wheelSeparation = 0.09f;

    private Vector3 targetVelocity;
    private float targetAngularVelocity;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, UpdateRaspimouseVelocity);

        Debug.Log($"Wheel Radius: {wheelRadius}, Wheel Separation: {wheelSeparation}");
        
        if (baseLink == null || leftWheel == null || rightWheel == null)
        {
            Debug.LogError("One or more ArticulationBody components are not assigned!");
        }
    }

    void UpdateRaspimouseVelocity(TwistMsg twist)
    {
        targetVelocity = new Vector3((float)twist.linear.x, 0, 0);
        targetAngularVelocity = (float)twist.angular.z;

        // Debug.Log($"Received: Linear X = {targetVelocity.x}, Angular Z = {targetAngularVelocity}");
    }

    void FixedUpdate()
    {
        ApplyVelocities();
    }

    void ApplyVelocities()
    {
        if (baseLink == null || leftWheel == null || rightWheel == null) return;

        float linearVelocity = Mathf.Clamp(targetVelocity.x, -maxLinearSpeed, maxLinearSpeed);
        float angularVelocity = Mathf.Clamp(targetAngularVelocity, -maxAngularSpeed, maxAngularSpeed);

        float leftWheelSpeed = (linearVelocity - angularVelocity * wheelSeparation / 2) / wheelRadius;
        float rightWheelSpeed = (linearVelocity + angularVelocity * wheelSeparation / 2) / wheelRadius;

        ApplyWheelSpeed(leftWheel, leftWheelSpeed);
        ApplyWheelSpeed(rightWheel, rightWheelSpeed);

        Vector3 localVelocity = new Vector3(0, 0, linearVelocity);
        baseLink.velocity = baseLink.transform.TransformDirection(localVelocity);
        baseLink.angularVelocity = new Vector3(0, -angularVelocity, 0);

        // Debug.Log($"Applied: Linear = {linearVelocity}, Angular = {angularVelocity}");
    }

    void ApplyWheelSpeed(ArticulationBody wheel, float speed)
    {
        var drive = wheel.xDrive;
        drive.target = speed * Mathf.Rad2Deg;
        wheel.xDrive = drive;
        // Debug.Log($"Applied to {wheel.name}: Speed = {speed}, Target = {drive.target}");
    }

    // void OnDrawGizmos()
    // {
    //     if (baseLink != null)
    //     {
    //         Gizmos.color = Color.blue;
    //         Gizmos.DrawRay(baseLink.transform.position, baseLink.velocity);
    //         Gizmos.color = Color.red;
    //         Gizmos.DrawRay(baseLink.transform.position, baseLink.angularVelocity);
    //     }
    // }
}