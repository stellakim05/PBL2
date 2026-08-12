// This file should be placed in Assets/Scripts/Metrics/MetricsDefinitions.cs
using UnityEngine;

namespace SystemEvaluation.Metrics
{
    [System.Serializable]
    public class SystemMetrics
    {
        // Temporal Performance
        public float rosUnityUpdateRate;
        public float hapticUpdateRate;
        public float forceResponseTime;
        public float controlInputDelay;
        public float stateUpdateLatency;

        // Network Performance
        public float messageDropRate;      // Lost messages percentage
        public float bandwidthUsage;       // Data transfer rate
        public float messageQueueLength;   // Message buffer status
        public float topicPublishFrequency;// ROS topic update rate
        public float syncErrorRate;        // Unity-ROS sync failures
    }

    [System.Serializable]
    public class ControlMetrics
    {
        // Control Precision
        public float averageRotationError;
        public float maxRotationDeviation;
        public float rotationStability;
        public float timeInTargetRange;
        public float positioningAccuracy;
        public float velocityPrecision;
        public float pathSmoothing;

        // Motion Quality
        public float accelerationProfile;    // Acceleration smoothness
        public float velocityOvershoot;      // Speed control precision
        public float settlingTime;           // Time to stable state
        public float steadyStateError;       // Persistent error
        public float disturbanceRejection;   // Response to external forces
    }

    [System.Serializable]
    public class HapticMetrics
    {
        // Haptic Metrics
        public float forceMagnitude;
        public float forceStability;
        public float forceResolution;
        public float contactResponseTime;
        public float contactReliability;
        public float contactPrecision;
    }

    public static class SuccessCriteria
    {
        // System Performance Thresholds
        public static class SystemThresholds
        {
            public const float MAX_LATENCY = 20.0f;           // milliseconds
            public const float MIN_UPDATE_RATE = 1000.0f;     // Hz
            public const float MAX_MESSAGE_DROP = 0.01f;      // 1%
            public const float MAX_SYNC_ERROR = 0.05f;        // 5%
            public const float MAX_RESOURCE_USAGE = 0.80f;    // 80%
        }

        // Control Performance Thresholds
        public static class ControlThresholds
        {
            public const float MAX_ROTATION_ERROR = 5.0f;     // degrees
            public const float MAX_POSITION_ERROR = 0.01f;    // meters
            public const float MIN_TIME_IN_RANGE = 0.90f;     // 90%
            public const float MAX_SETTLING_TIME = 2.0f;      // seconds
            public const float MAX_STEADY_STATE_ERROR = 1.0f; // degree
        }

        // Haptic Performance Thresholds
        public static class HapticThresholds
        {
            public const float MIN_FORCE_RESOLUTION = 0.1f;    // Newtons
            public const float MAX_FORCE_LATENCY = 0.01f;      // seconds
            public const float MAX_FORCE_ERROR = 0.05f;        // 5%
            public const float MIN_CONTACT_RELIABILITY = 0.95f;// 95%
            public const float MAX_FORCE_RIPPLE = 0.02f;       // 2%
        }

        // Performance Levels
        public enum PerformanceLevel
        {
            Excellent,    // Exceeds all thresholds
            Good,         // Meets all thresholds
            Acceptable,   // Meets critical thresholds
            Poor,        // Fails some critical thresholds
            Unacceptable // Fails multiple critical thresholds
        }
    }
}