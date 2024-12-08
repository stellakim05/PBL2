using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; 
using RosMessageTypes.Std; 

public class QuadrotorSubscriber : MonoBehaviour
{
    private ROSConnection ros; 
    public string topicName = "/quadrotor/cmd_vel"; 

    public ArticulationBody baseLink; // The articulation body of the quadrotor

    public float maxLinearSpeed = 0.2f; // Maximum linear speed for the quadrotor
    public float maxAngularSpeed = 1.0f; // Maximum angular speed for the quadrotor

    private Vector3 targetVelocity; // Target linear velocity
    private float targetAngularVelocity; // Target angular velocity

    private void Start()
    {
        // Get or create the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the feedback topic that provides velocity data
        ros.Subscribe<TwistMsg>(topicName, UpdateQuadrotorVelocity);
        Debug.Log("Connected to ROS!");

        if (baseLink == null) {
            Debug.LogError("The articulation component is not assigned!");
        }
    }

    // Callback method to process the received Twist message
    private void UpdateQuadrotorVelocity(TwistMsg message)
    {
        targetVelocity = new Vector3((float)message.linear.x, (float)message.linear.y, (float)message.linear.z);
        targetAngularVelocity = (float)message.angular.z;

        Debug.Log($"Received: Linear X: = {message.linear.x}, Angular Z = {targetAngularVelocity}");
        
        BoolMsg feedbackMsg = new BoolMsg(targetVelocity.magnitude > 0 || targetAngularVelocity != 0);
        ros.Publish("/unity_movement_feedback", feedbackMsg);
    }

    void FixedUpdate() {
        ApplyVelocityToQuadrotor();
    }

    private void ApplyVelocityToQuadrotor() {
        if (baseLink == null) return;

        float linearVelocity = Mathf.Clamp(targetVelocity.x,  -maxLinearSpeed, maxLinearSpeed);
        float angularVelocity = Mathf.Clamp(targetAngularVelocity, -maxAngularSpeed, maxAngularSpeed);

        baseLink.velocity = baseLink.transform.TransformDirection(new Vector3(0, targetVelocity.y, linearVelocity));
        baseLink.angularVelocity = new Vector3(0, angularVelocity, 0);
    }
}
