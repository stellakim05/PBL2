using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class RaspimouseTogglePublisher : MonoBehaviour
{
    private ROSConnection ros;
    private ROSPerformanceMonitor performanceMonitor;
    private bool isMoving = false;
    private bool isInitialized = false;
    private float publishRate = 0.01f; // 100Hz publish rate
    private float timeSinceLastPublish = 0f;

    // References to both robots' articulation bodies
    public ArticulationBody raspimouse1Base;
    public ArticulationBody raspimouse2Base;
    private Vector3 lastVelocity1 = Vector3.zero;
    private Vector3 lastVelocity2 = Vector3.zero;

    void Start()
    {
        // Validate articulation body references
        if (raspimouse1Base == null || raspimouse2Base == null)
        {
            Debug.LogError("Raspimouse base ArticulationBody references not set!");
            return;
        }

        StartCoroutine(WaitForROSConnection());
    }

    private System.Collections.IEnumerator WaitForROSConnection()
        {
            // Use GetOrCreateInstance instead of instance
            while (ROSConnection.GetOrCreateInstance() == null)
            {
                yield return new WaitForSeconds(0.1f);
            }

            // Get ROS connection instance using new method
            ros = ROSConnection.GetOrCreateInstance();
            yield return null;

            performanceMonitor = ros.gameObject.GetComponent<ROSPerformanceMonitor>();
            if (performanceMonitor == null)
            {
                Debug.LogError($"Could not find ROSPerformanceMonitor on ROSConnection GameObject");
            }

            ros.RegisterPublisher<TwistMsg>("/raspimouse1/cmd_vel");
            ros.RegisterPublisher<TwistMsg>("/raspimouse2/cmd_vel");
            ros.RegisterPublisher<TwistMsg>("/raspimouse1/state");
            ros.RegisterPublisher<TwistMsg>("/raspimouse2/state");
            ros.RegisterPublisher<BoolMsg>("/unity_movement_feedback");
            
            isInitialized = true;
            Debug.Log("Publisher fully initialized");
        }

    void Update()
    {
        if (!isInitialized) return;

        if (Input.GetKeyDown(KeyCode.Space))
        {
            ToggleMovement();
        }

        // Continuous state publishing at specified rate
        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish >= publishRate)
        {
            PublishRobotStates();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishRobotStates()
    {
        // Publish state for Raspimouse 1
        TwistMsg state1 = new TwistMsg
        {
            linear = new Vector3Msg(
                raspimouse1Base.velocity.x,
                raspimouse1Base.velocity.y,
                raspimouse1Base.velocity.z
            ),
            angular = new Vector3Msg(
                raspimouse1Base.angularVelocity.x,
                raspimouse1Base.angularVelocity.y,
                raspimouse1Base.angularVelocity.z
            )
        };

        // Publish state for Raspimouse 2
        TwistMsg state2 = new TwistMsg
        {
            linear = new Vector3Msg(
                raspimouse2Base.velocity.x,
                raspimouse2Base.velocity.y,
                raspimouse2Base.velocity.z
            ),
            angular = new Vector3Msg(
                raspimouse2Base.angularVelocity.x,
                raspimouse2Base.angularVelocity.y,
                raspimouse2Base.angularVelocity.z
            )
        };

        // Only publish if velocity has changed
        if (lastVelocity1 != raspimouse1Base.velocity)
        {
            PublishAndTrack("/raspimouse1/state", state1);
            lastVelocity1 = raspimouse1Base.velocity;
        }

        if (lastVelocity2 != raspimouse2Base.velocity)
        {
            PublishAndTrack("/raspimouse2/state", state2);
            lastVelocity2 = raspimouse2Base.velocity;
        }
    }

    void ToggleMovement()
    {
        isMoving = !isMoving;
        Debug.Log($"Unity: Toggling movement to {isMoving}");

        TwistMsg msg = new TwistMsg();
        msg.linear.z = isMoving ? 0.2f : 0.0f;

        PublishAndTrack("/raspimouse1/cmd_vel", msg);
        PublishAndTrack("/raspimouse2/cmd_vel", msg);

        BoolMsg feedbackMsg = new BoolMsg(isMoving);
        PublishAndTrackFeedback("/unity_movement_feedback", feedbackMsg);
    }

    private void PublishAndTrack<T>(string topic, T message) where T : Message
    {
        Debug.Log($"Publishing to topic: {topic}");
        ros.Publish(topic, message);

        if (performanceMonitor != null)
        {
            performanceMonitor.OnMessagePublished(topic);
            Debug.Log($"Message tracked for topic: {topic}");
        }
    }

    private void PublishAndTrackFeedback(string topic, BoolMsg message)
    {
        Debug.Log($"Publishing feedback to topic: {topic}, value: {message.data}");
        ros.Publish(topic, message);

        if (performanceMonitor != null)
        {
            performanceMonitor.OnMessagePublished(topic);
            Debug.Log($"Feedback message tracked for topic: {topic}");
        }
    }
}