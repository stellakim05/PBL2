using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using System.IO;
using System.Collections.Generic;
// ROSPerformanceMonitor.cs
public class ROSPerformanceMonitor : MonoBehaviour
{
    private ROSConnection rosConnection;
    private TechnicalPerformanceDataCollector dataCollector;
    private float lastCheckTime;
    [SerializeField] private float monitoringInterval = 0.05f;

    // Track only the most recent command for each robot
    private Dictionary<string, (float timestamp, float velocity)> lastCommands = 
        new Dictionary<string, (float timestamp, float velocity)>();

    private readonly string[] monitoredTopics = new string[] {
        "/raspimouse1/cmd_vel",
        "/raspimouse2/cmd_vel",
        "/raspimouse1/state",
        "/raspimouse2/state",
        "/unity_movement_feedback"
    };

    void Start()
    {
        rosConnection = GetComponent<ROSConnection>();
        dataCollector = GetComponent<TechnicalPerformanceDataCollector>();

        if (dataCollector == null)
        {
            dataCollector = gameObject.AddComponent<TechnicalPerformanceDataCollector>();
        }

        if (rosConnection.ConnectOnStart)
        {
            dataCollector.StartRecording();
        }

        SetupTopicMonitoring();
    }

    private void SetupTopicMonitoring()
    {
        foreach (string topic in monitoredTopics)
        {
            if (topic.Contains("cmd_vel"))
            {
                rosConnection.Subscribe<TwistMsg>(topic, msg => OnCommandReceived(topic, msg));
            }
            else if (topic.Contains("state"))
            {
                rosConnection.Subscribe<TwistMsg>(topic, msg => OnStateReceived(topic, msg));
            }
            else if (topic.Contains("feedback"))
            {
                rosConnection.Subscribe<BoolMsg>(topic, msg => OnFeedbackReceived(topic, msg));
            }
        }
    }

    private void OnCommandReceived(string topic, TwistMsg msg)
    {
        string robotKey = topic.Split('/')[1];  // Get "raspimouse1" or "raspimouse2"
        float currentTime = Time.realtimeSinceStartup;
        
        // Only update if velocity actually changed
        if (!lastCommands.ContainsKey(robotKey) || 
            lastCommands[robotKey].velocity != (float)msg.linear.x)
        {
            lastCommands[robotKey] = (currentTime, (float)msg.linear.x);
            Debug.Log($"[COMMAND] New command velocity {msg.linear.x} at {currentTime:F3}s for {robotKey}");
        }
    }
    
     private void OnStateReceived(string topic, TwistMsg msg)
    {
        string robotKey = topic.Split('/')[1];
        float currentTime = Time.realtimeSinceStartup;

        if (lastCommands.TryGetValue(robotKey, out var commandData))
        {
            float latency = currentTime - commandData.timestamp;
            
            // Only process if this is a recent state change
            if (latency <= 10000.0f)  // 2 second threshold
            {
                Debug.Log($"[TIMING] State received for {robotKey} - Command Time: {commandData.timestamp:F3}s, " +
                         $"Current Time: {currentTime:F3}s, Latency: {latency:F6}s");
                dataCollector.UpdateLatency(latency, latency);
                
                // Clear the command after processing to prevent stale measurements
                lastCommands.Remove(robotKey);
            }
        }
    }

    private void OnFeedbackReceived(string topic, BoolMsg msg)
    {
        dataCollector.OnMessageReceived("feedback", Time.realtimeSinceStartup);
    }

    public void OnMessagePublished(string topic)
    {
        dataCollector.OnMessageReceived(topic, Time.realtimeSinceStartup);
    }

    void Update()
    {
        if (Time.time - lastCheckTime < monitoringInterval) return;
        lastCheckTime = Time.time;

        if (rosConnection != null && !rosConnection.HasConnectionError)
        {
            dataCollector.UpdateMetrics();
        }
    }
}