using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class MultiObjectCamera : MonoBehaviour
{
    [Header("Target Settings")]
    public List<Transform> targets = new List<Transform>();
    public float minDistance = 5f;     // Minimum distance from targets
    public float maxDistance = 20f;    // Maximum distance from targets
    public float heightOffset = 2f;    // How high above targets to position camera
    public float followSpeed = 5f;     // How fast camera moves to new position
    
    [Header("View Settings")]
    public float minFOV = 40f;         // Minimum field of view
    public float maxFOV = 90f;         // Maximum field of view
    public float targetPadding = 2f;   // Extra space around targets in view
    public float smoothTime = 0.5f;    // Smoothing time for camera movement

    [Header("Follow Angle Settings")]
    [Tooltip("Angle around Y axis (0 = behind, 90 = right side, 180 = front, -90 = left side)")]
    public float horizontalAngle = 0f;
    [Tooltip("Vertical angle (0 = horizontal, positive = look down, negative = look up)")]
    [Range(-89f, 89f)]
    public float verticalAngle = 45f;
    public bool smoothRotation = true;
    public float rotationSpeed = 5f;
    
    [Header("Orbit Settings")]
    public bool enableOrbit = false;   // Enable camera orbiting
    public float orbitSpeed = 20f;     // Degrees per second
    public float orbitRadius = 10f;    // Distance from center while orbiting

    // Private variables for smoothing
    private Vector3 currentVelocity;
    private float currentFOVVelocity;
    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private float targetFOV;
    private float orbitAngle;
    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            Debug.LogError("Camera component not found!");
            enabled = false;
            return;
        }

        // Initialize position and rotation
        if (targets.Count > 0)
        {
            UpdateCameraPosition(true); // Force immediate update
        }
    }

    void LateUpdate()
    {
        // Remove any null targets
        targets.RemoveAll(t => t == null);
        
        if (targets.Count == 0) return;

        UpdateCameraPosition(false);
        UpdateFieldOfView();
    }

    void UpdateCameraPosition(bool immediate = false)
    {
        Vector3 centerPoint = GetCenterPoint();
        Vector3 desiredPosition;

        if (enableOrbit)
        {
            // Update orbit angle
            orbitAngle += orbitSpeed * Time.deltaTime;
            float totalAngle = orbitAngle + horizontalAngle;
            
            // Calculate position using orbit and vertical angle
            float horizontalRadius = orbitRadius * Mathf.Cos(verticalAngle * Mathf.Deg2Rad);
            float height = orbitRadius * Mathf.Sin(verticalAngle * Mathf.Deg2Rad);
            
            float rad = totalAngle * Mathf.Deg2Rad;
            Vector3 offset = new Vector3(
                horizontalRadius * Mathf.Sin(rad),
                height + heightOffset,
                horizontalRadius * Mathf.Cos(rad)
            );
            
            desiredPosition = centerPoint + offset;
        }
        else
        {
            // Calculate distance based on targets spread
            float targetDistance = Mathf.Clamp(
                GetMaxTargetDistance() + targetPadding,
                minDistance,
                maxDistance
            );

            // Calculate horizontal position
            float horizontalRad = horizontalAngle * Mathf.Deg2Rad;
            float horizontalDistance = targetDistance * Mathf.Cos(verticalAngle * Mathf.Deg2Rad);
            
            // Calculate offset from center point
            Vector3 offset = new Vector3(
                horizontalDistance * Mathf.Sin(horizontalRad),
                targetDistance * Mathf.Sin(verticalAngle * Mathf.Deg2Rad) + heightOffset,
                horizontalDistance * Mathf.Cos(horizontalRad)
            );

            desiredPosition = centerPoint + offset;
        }

        // Calculate desired rotation to look at center point
        Quaternion desiredRotation = Quaternion.LookRotation(centerPoint - desiredPosition);

        if (immediate)
        {
            // Set position and rotation immediately
            transform.position = desiredPosition;
            transform.rotation = desiredRotation;
        }
        else
        {
            // Smoothly move camera
            transform.position = Vector3.SmoothDamp(
                transform.position,
                desiredPosition,
                ref currentVelocity,
                smoothTime
            );

            // Smoothly rotate camera
            if (smoothRotation)
            {
                transform.rotation = Quaternion.Slerp(
                    transform.rotation,
                    desiredRotation,
                    Time.deltaTime * rotationSpeed
                );
            }
            else
            {
                transform.rotation = desiredRotation;
            }
        }
    }

    void UpdateFieldOfView()
    {
        float requiredFOV = CalculateRequiredFOV();
        targetFOV = Mathf.Clamp(requiredFOV, minFOV, maxFOV);
        
        cam.fieldOfView = Mathf.SmoothDamp(
            cam.fieldOfView,
            targetFOV,
            ref currentFOVVelocity,
            smoothTime
        );
    }

    Vector3 GetCenterPoint()
    {
        if (targets.Count == 1)
        {
            return targets[0].position;
        }

        var bounds = new Bounds(targets[0].position, Vector3.zero);
        foreach (Transform target in targets)
        {
            bounds.Encapsulate(target.position);
        }

        return bounds.center;
    }

    float GetMaxTargetDistance()
    {
        if (targets.Count == 0) return 0;
        if (targets.Count == 1) return minDistance;

        float maxDistance = 0f;
        for (int i = 0; i < targets.Count; i++)
        {
            for (int j = i + 1; j < targets.Count; j++)
            {
                float distance = Vector3.Distance(
                    targets[i].position,
                    targets[j].position
                );
                maxDistance = Mathf.Max(maxDistance, distance);
            }
        }

        return maxDistance;
    }

    float CalculateRequiredFOV()
    {
        if (targets.Count <= 1) return minFOV;

        float distance = Vector3.Distance(transform.position, GetCenterPoint());
        float maxTargetDistance = GetMaxTargetDistance();

        float requiredHalfFOV = Mathf.Atan2(
            (maxTargetDistance + targetPadding) * 0.5f,
            distance
        );

        return Mathf.Rad2Deg * requiredHalfFOV * 2;
    }

    // Public methods for managing targets
    public void AddTarget(Transform target)
    {
        if (target != null && !targets.Contains(target))
        {
            targets.Add(target);
        }
    }

    public void RemoveTarget(Transform target)
    {
        targets.Remove(target);
    }

    public void ClearTargets()
    {
        targets.Clear();
    }

    // Method to set camera angle
    public void SetCameraAngle(float horizontal, float vertical)
    {
        horizontalAngle = horizontal;
        verticalAngle = Mathf.Clamp(vertical, -89f, 89f);
    }

    void OnDrawGizmos()
    {
        if (!enabled || targets.Count == 0) return;

        Gizmos.color = Color.yellow;
        Vector3 centerPoint = GetCenterPoint();
        foreach (Transform target in targets)
        {
            if (target != null)
            {
                Gizmos.DrawLine(transform.position, target.position);
            }
        }

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(centerPoint, 0.5f);

        // Draw angle direction
        Gizmos.color = Color.blue;
        float horizontalRad = horizontalAngle * Mathf.Deg2Rad;
        Vector3 angleDirection = new Vector3(
            Mathf.Sin(horizontalRad),
            Mathf.Tan(verticalAngle * Mathf.Deg2Rad),
            Mathf.Cos(horizontalRad)
        ).normalized * 2f;
        Gizmos.DrawRay(centerPoint, angleDirection);
    }
}