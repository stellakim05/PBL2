using UnityEngine;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

public class TechnicalPerformanceDataCollector : MonoBehaviour
{
    [SerializeField] private string customSessionPrefix = "";
    [SerializeField] private float samplingRate = 0.02f; // 50Hz sampling
    
    private bool isRecording;
    private string sessionID;
    private string dataPath;
    private StreamWriter systemWriter;
    private float sessionStartTime;
    private float lastSampleTime;

    // Message rate tracking
    private float messageRateWindow = 1.0f;
    private float lastRateCalculationTime;
    private int messagesInCurrentWindow;
    private float currentMessageRate;

    // Latency tracking
    private Queue<float> latencyWindow = new Queue<float>();
    private Queue<float> responseTimeWindow = new Queue<float>();
    private const int WINDOW_SIZE = 50;
    private float currentAverageLatency;
    private float currentResponseTime;
    private float lastResponseTime = 0f;  // Add this to track individual responses

    public void StartRecording()
    {
        if (isRecording) return;

        sessionID = $"{customSessionPrefix}_{DateTime.Now:yyyyMMdd_HHmmss}";
        dataPath = Path.Combine(Application.dataPath, "ExperimentData");
        Directory.CreateDirectory(dataPath);

        InitializeSystemLog();
        ResetMetrics();

        isRecording = true;
        sessionStartTime = Time.realtimeSinceStartup;
        lastSampleTime = Time.realtimeSinceStartup;
        lastRateCalculationTime = Time.realtimeSinceStartup;
    }

    private void InitializeSystemLog()
    {
        string filename = Path.Combine(dataPath, $"system_performance_{sessionID}.csv");
        systemWriter = new StreamWriter(filename, false);
        systemWriter.WriteLine("Timestamp,AverageLatency,MessageRate");
    }

    private void ResetMetrics()
    {
        messagesInCurrentWindow = 0;
        currentMessageRate = 0;
        currentAverageLatency = 0;
        currentResponseTime = 0;
        latencyWindow.Clear();
        responseTimeWindow.Clear();
    }

    public void OnMessageReceived(string type, float timestamp)
    {
        if (!isRecording) return;

        messagesInCurrentWindow++;
        UpdateMessageRate(timestamp);
    }

     public void UpdateLatency(float responseTime, float latency)
    {
        if (!isRecording) return;

        // Only record immediate response times
        if (responseTime > 0 && responseTime <= 100.0f)
        {
            lastResponseTime = responseTime;
            currentResponseTime = responseTime;  // Don't average, just use current

            Debug.Log($"[LATENCY] Updated - Response Time: {responseTime:F6}s");
        }
    }

    private void UpdateMessageRate(float currentTime)
    {
        if (currentTime - lastRateCalculationTime >= messageRateWindow)
        {
            currentMessageRate = messagesInCurrentWindow / messageRateWindow;
            messagesInCurrentWindow = 0;
            lastRateCalculationTime = currentTime;
            Debug.Log($"Message Rate: {currentMessageRate:F2} msg/s");
        }
    }

    public void UpdateMetrics()
    {
        if (!isRecording) return;

        float currentTime = Time.realtimeSinceStartup;
        if (currentTime - lastSampleTime >= samplingRate)
        {
            RecordMetrics();
            lastSampleTime = currentTime;
        }
    }

    private void RecordMetrics()
    {
        if (systemWriter == null) return;

        float timestamp = Time.realtimeSinceStartup - sessionStartTime;
        
        // Only record if we have a recent response time
        float responseToRecord = lastResponseTime;
        if (Time.realtimeSinceStartup - lastSampleTime > 2.0f)
        {
            responseToRecord = 0;  // Reset if no recent measurements
        }

        string record = string.Format("{0:F6},{1:F6},{2:F2}",
            timestamp,
            responseToRecord,
            currentMessageRate
        );

        systemWriter.WriteLine(record);
        systemWriter.Flush();
    }

    public void StopRecording()
    {
        if (!isRecording) return;
        isRecording = false;
        
        if (systemWriter != null)
        {
            systemWriter.Close();
            systemWriter = null;
        }
    }

    private void OnDisable()
    {
        if (isRecording)
        {
            StopRecording();
        }
    }
}