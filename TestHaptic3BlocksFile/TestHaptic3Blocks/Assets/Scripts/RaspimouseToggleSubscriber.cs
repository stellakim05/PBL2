using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RaspimouseToggleSubscriber : MonoBehaviour
{
    ROSConnection ros;
    private ROSPerformanceMonitor performanceMonitor;
    public string cmdVelTopic = "/raspimouse1/cmd_vel";
    public string stateTopic = "/raspimouse1/state";

    public ArticulationBody baseLink;
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    public float maxLinearSpeed = 0.2f;  // Max 0.2 m/s
    public float maxAngularSpeed = 1.0f;
    public float wheelRadius = 0.024f;
    public float wheelSeparation = 0.09f;

    // Add acceleration control
    public float accelerationRate = 0.1f;  // Adjust this value to control how quickly it reaches max speed
    private Vector3 currentVelocity = Vector3.zero;
    private float currentAngularVelocity = 0f;
    private Vector3 targetVelocity;
    private float targetAngularVelocity;
    // Add state tracking
    private bool isMoving = false;
    private bool isInitialized = false;
    private float lastStatePublishTime = 0f;
    private float statePublishRate = 0.005f; // 50Hz state publishing

    void Start()
    {
        StartCoroutine(WaitForROSConnection());
    }

    private System.Collections.IEnumerator WaitForROSConnection()
    {
        // Wait for ROSConnection to be created
        while (ROSConnection.GetOrCreateInstance() == null)
        {
            yield return new WaitForSeconds(0.1f);
        }

        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Wait one frame to ensure all components are initialized
        yield return null;

        // Find performance monitor on the ROSConnection GameObject
        performanceMonitor = ros.gameObject.GetComponent<ROSPerformanceMonitor>();
        if (performanceMonitor == null)
        {
            Debug.LogError($"Could not find ROSPerformanceMonitor on ROSConnection GameObject: {ros.gameObject.name}");
        }
        else
        {
            Debug.Log("Successfully found ROSPerformanceMonitor");
        }

        // Subscribe to topics
        ros.Subscribe<TwistMsg>(cmdVelTopic, UpdateRaspimouseVelocity);

        Debug.Log($"Subscriber initialized. Performance Monitor status: " + (performanceMonitor != null ? "Found" : "Not Found"));
        Debug.Log($"Wheel Radius: {wheelRadius}, Wheel Separation: {wheelSeparation}");

        if (baseLink == null || leftWheel == null || rightWheel == null)
        {
            Debug.LogError("One or more ArticulationBody components are not assigned!");
            yield break;
        }

        // Subscribe to command topics
        ros.Subscribe<TwistMsg>(cmdVelTopic, UpdateRaspimouseVelocity);
        
        // Register state publisher
        ros.RegisterPublisher<TwistMsg>(stateTopic);

        isInitialized = true;
        Debug.Log("Subscriber fully initialized with state publishing");
    }

    void UpdateRaspimouseVelocity(TwistMsg twist)
    {
        if (!isInitialized) return;

        // Clamp the incoming velocity commands
        float linearX = Mathf.Clamp((float)twist.linear.x, -maxLinearSpeed, maxLinearSpeed);
        float angularZ = Mathf.Clamp((float)twist.angular.z, -maxAngularSpeed, maxAngularSpeed);

        targetVelocity = new Vector3(linearX, 0, 0);
        targetAngularVelocity = angularZ;

        // Check movement state
        bool shouldBeMoving = targetVelocity.magnitude > 0 || targetAngularVelocity != 0;
        if (shouldBeMoving != isMoving)
        {
            isMoving = shouldBeMoving;
            SendFeedback(isMoving);
        }
    }

    private void SendFeedback(bool moving)
    {
        Debug.Log($"Sending movement feedback: {moving}");
        BoolMsg feedbackMsg = new BoolMsg(moving);
        ros.Publish("/unity_movement_feedback", feedbackMsg);

        if (performanceMonitor != null)
        {
            performanceMonitor.OnMessagePublished("/unity_movement_feedback");
            Debug.Log($"Tracked feedback message: moving = {moving}");
        }
    }

    void FixedUpdate()
    {
        // Smoothly interpolate to target velocities
        currentVelocity = Vector3.MoveTowards(
            currentVelocity, 
            targetVelocity, 
            accelerationRate * Time.fixedDeltaTime
        );

        currentAngularVelocity = Mathf.MoveTowards(
            currentAngularVelocity, 
            targetAngularVelocity, 
            accelerationRate * Time.fixedDeltaTime
        );

        // Publish state at regular intervals
        if (Time.time - lastStatePublishTime >= statePublishRate)
        {
            PublishCurrentState();
            lastStatePublishTime = Time.time;
        }

        ApplyVelocities();
    }

    void PublishCurrentState()
    {
        if (!isInitialized || baseLink == null) return;

        TwistMsg stateMsg = new TwistMsg
        {
            linear = new Vector3Msg(
                baseLink.velocity.x,
                baseLink.velocity.y,
                baseLink.velocity.z
            ),
            angular = new Vector3Msg(
                baseLink.angularVelocity.x,
                baseLink.angularVelocity.y,
                baseLink.angularVelocity.z
            )
        };

        ros.Publish(stateTopic, stateMsg);

        if (performanceMonitor != null)
        {
            performanceMonitor.OnMessagePublished(stateTopic);
        }
    }

    void ApplyVelocities()
    {
        if (baseLink == null || leftWheel == null || rightWheel == null) return;

        // Use current (interpolated) velocities instead of target velocities directly
        float linearVelocity = Mathf.Clamp(currentVelocity.x, -maxLinearSpeed, maxLinearSpeed);
        float angularVelocity = Mathf.Clamp(currentAngularVelocity, -maxAngularSpeed, maxAngularSpeed);

        // Calculate wheel speeds
        float leftWheelSpeed = (linearVelocity - angularVelocity * wheelSeparation / 2) / wheelRadius;
        float rightWheelSpeed = (linearVelocity + angularVelocity * wheelSeparation / 2) / wheelRadius;

        // Apply wheel speeds
        ApplyWheelSpeed(leftWheel, leftWheelSpeed);
        ApplyWheelSpeed(rightWheel, rightWheelSpeed);

        // Apply body velocities
        Vector3 localVelocity = new Vector3(0, 0, linearVelocity);
        baseLink.velocity = baseLink.transform.TransformDirection(localVelocity);
        baseLink.angularVelocity = new Vector3(0, -angularVelocity, 0);

        // Log actual velocities for debugging
        // Debug.Log($"Current Velocities - Linear: {linearVelocity:F3}, Angular: {angularVelocity:F3}");
    }

    void ApplyWheelSpeed(ArticulationBody wheel, float speed)
    {
        var drive = wheel.xDrive;
        drive.target = Mathf.Clamp(speed * Mathf.Rad2Deg, -maxLinearSpeed * Mathf.Rad2Deg, maxLinearSpeed * Mathf.Rad2Deg);
        wheel.xDrive = drive;
    }

    // Add method to reset velocities when needed
    public void ResetVelocities()
    {
        currentVelocity = Vector3.zero;
        currentAngularVelocity = 0f;
        targetVelocity = Vector3.zero;
        targetAngularVelocity = 0f;
    }
}