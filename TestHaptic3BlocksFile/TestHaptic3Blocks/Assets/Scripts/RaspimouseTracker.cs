using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;

[Serializable]
public class RaspimouseTransformData
{
    public float timestamp;          // Time since tracking started
    public Vector3 position;         // Global position
    public Vector3 rotation;         // Euler angles
    public Vector3 velocity;         // Velocity vector
    public float speed;             // Current speed
    public float angularSpeed;      // Angular speed
    public float distanceTraveled;  // Cumulative distance from start position

    public RaspimouseTransformData()
    {
        timestamp = 0f;
        position = Vector3.zero;
        rotation = Vector3.zero;
        velocity = Vector3.zero;
        speed = 0f;
        angularSpeed = 0f;
        distanceTraveled = 0f;
    }
}

public class RaspimouseTracker : MonoBehaviour
{
    [Header("Tracking Settings")]
    public float samplingRate = 0.1f; // How often to record data (seconds)
    
    [Header("Debug Settings")]
    public bool showDebugInfo = true;

    // Runtime tracking
    private float sessionStartTime;
    private float lastSampleTime;
    private Vector3 lastPosition;
    private Vector3 lastRotation;
    private Vector3 startPosition;
    private float totalDistance;
    private string sessionFilePath;
    private bool isTracking = false;

    // Data storage
    private List<RaspimouseTransformData> sessionData;
    public int totalSamples { get; private set; }

    private void OnEnable()
    {
        StartNewSession();
    }

    private void StartNewSession()
    {
        // Initialize tracking variables
        sessionStartTime = Time.time;
        lastSampleTime = Time.time;
        lastPosition = transform.position;
        lastRotation = transform.eulerAngles;
        startPosition = transform.position;
        totalDistance = 0f;
        totalSamples = 0;
        isTracking = true;

        // Initialize data storage
        sessionData = new List<RaspimouseTransformData>();

        // Setup data file
        string dataFolder = Path.Combine(Application.dataPath, "RaspimouseData");
        if (!Directory.Exists(dataFolder))
        {
            Directory.CreateDirectory(dataFolder);
        }

        // Create new session file with timestamp
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        sessionFilePath = Path.Combine(dataFolder, $"raspimouse_session_{timestamp}.csv");

        // Write header to file
        using (StreamWriter writer = new StreamWriter(sessionFilePath, false))
        {
            writer.WriteLine("Timestamp,PosX,PosY,PosZ,RotX,RotY,RotZ,VelX,VelY,VelZ,Speed,AngularSpeed,DistanceFromStart,TotalDistance");
        }

        if (showDebugInfo)
        {
            Debug.Log($"Started new tracking session: {sessionFilePath}");
        }
    }

    private void Update()
    {
        if (!isTracking) return;

        // Check if it's time to record data
        if (Time.time - lastSampleTime >= samplingRate)
        {
            RecordTransformData();
            lastSampleTime = Time.time;
        }
    }

    private void RecordTransformData()
    {
        RaspimouseTransformData data = new RaspimouseTransformData();
        
        // Basic transform data
        data.timestamp = Time.time - sessionStartTime;
        data.position = transform.position;
        data.rotation = transform.eulerAngles;

        // Calculate velocity and speed
        Vector3 displacement = transform.position - lastPosition;
        data.velocity = displacement / samplingRate;
        data.speed = data.velocity.magnitude;

        // Calculate angular velocity
        Vector3 rotationDelta = transform.eulerAngles - lastRotation;
        // Normalize angle differences to -180 to 180 range
        rotationDelta.x = NormalizeAngle(rotationDelta.x);
        rotationDelta.y = NormalizeAngle(rotationDelta.y);
        rotationDelta.z = NormalizeAngle(rotationDelta.z);
        data.angularSpeed = rotationDelta.magnitude / samplingRate;

        // Update distance calculations
        float distanceThisFrame = displacement.magnitude;
        totalDistance += distanceThisFrame;
        data.distanceTraveled = totalDistance;

        // Save data
        sessionData.Add(data);
        totalSamples++;

        // Save to file
        SaveDataPoint(data);

        // Update last known positions
        lastPosition = transform.position;
        lastRotation = transform.eulerAngles;

        if (showDebugInfo && totalSamples % 10 == 0) // Show debug every 10 samples
        {
            Debug.Log($"Position: {data.position}, Speed: {data.speed:F2} m/s, Total Distance: {totalDistance:F2}m");
        }
    }

    private float NormalizeAngle(float angle)
    {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void SaveDataPoint(RaspimouseTransformData data)
    {
        try
        {
            using (StreamWriter writer = new StreamWriter(sessionFilePath, true))
            {
                float distanceFromStart = Vector3.Distance(startPosition, data.position);
                writer.WriteLine(
                    $"{data.timestamp:F3}," +
                    $"{data.position.x:F3},{data.position.y:F3},{data.position.z:F3}," +
                    $"{data.rotation.x:F3},{data.rotation.y:F3},{data.rotation.z:F3}," +
                    $"{data.velocity.x:F3},{data.velocity.y:F3},{data.velocity.z:F3}," +
                    $"{data.speed:F3}," +
                    $"{data.angularSpeed:F3}," +
                    $"{distanceFromStart:F3}," +
                    $"{totalDistance:F3}"
                );
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to write to file: {e.Message}");
            isTracking = false; // Stop tracking if we can't write to file
        }
    }

    public float GetTotalDistance()
    {
        return totalDistance;
    }

    public float GetAverageSpeed()
    {
        if (sessionData.Count == 0) return 0f;

        float totalSpeed = 0f;
        foreach (var data in sessionData)
        {
            totalSpeed += data.speed;
        }
        return totalSpeed / sessionData.Count;
    }

    private void OnDisable()
    {
        StopSession();
    }

    private void StopSession()
    {
        if (!isTracking) return;

        isTracking = false;
        
        // Log final statistics
        float averageSpeed = GetAverageSpeed();
        float duration = Time.time - sessionStartTime;
        
        Debug.Log($"Session ended - Duration: {duration:F1}s, Samples: {totalSamples}, " +
                 $"Distance: {totalDistance:F2}m, Avg Speed: {averageSpeed:F2}m/s");
        
        // Write session summary to file
        try
        {
            using (StreamWriter writer = new StreamWriter(sessionFilePath, true))
            {
                writer.WriteLine("\nSession Summary:");
                writer.WriteLine($"Duration: {duration:F1} seconds");
                writer.WriteLine($"Total Samples: {totalSamples}");
                writer.WriteLine($"Total Distance: {totalDistance:F2} meters");
                writer.WriteLine($"Average Speed: {averageSpeed:F2} m/s");
                writer.WriteLine($"Start Position: {startPosition}");
                writer.WriteLine($"End Position: {transform.position}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to write session summary: {e.Message}");
        }
    }

    private void OnApplicationQuit()
    {
        StopSession();
    }
}