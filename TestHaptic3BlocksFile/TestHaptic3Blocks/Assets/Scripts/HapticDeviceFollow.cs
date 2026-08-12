using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class HapticDeviceFollow : MonoBehaviour
{
    [Header("Target Settings")]
    public List<Transform> targets = new List<Transform>();
    public string targetTag = "Robot";
    public bool autoFindTargets = true;
    
    [Header("Position Settings")]
    [Tooltip("Offset from target(s) in meters:\nx: left/right\ny: up/down\nz: forward/back")]
    public Vector3 offset = new Vector3(0, 0.7f, 0.2f);

    [Header("Pen Orientation")]
    [Tooltip("Fixed rotation for the haptic pen")]
    public Vector3 fixedRotation = new Vector3(0, 0, 0);
    public bool useFixedRotation = true;

    [Header("Follow Settings")]
    public float positionSmoothSpeed = 5f;
    public bool showDebugVisuals = true;

    // Private variables
    private float nextSearchTime;
    private Vector3 currentVelocity;
    private Transform currentTarget;
    private HapticPlugin hapticPlugin;
    private Quaternion initialRotation;

    private void Start()
    {
        hapticPlugin = GetComponent<HapticPlugin>();
        
        if (hapticPlugin == null)
        {
            Debug.LogError("HapticPlugin not found on this GameObject!");
            return;
        }

        // Store initial rotation
        initialRotation = transform.rotation;

        // Set initial fixed rotation if using it
        if (useFixedRotation)
        {
            transform.rotation = Quaternion.Euler(fixedRotation);
        }

        if (autoFindTargets)
        {
            FindTargets();
        }

        UpdateCurrentTarget();
    }

    private void FindTargets()
    {
        targets.Clear();
        GameObject[] taggedObjects = GameObject.FindGameObjectsWithTag(targetTag);
        foreach (GameObject obj in taggedObjects)
        {
            targets.Add(obj.transform);
        }
        Debug.Log($"Found {targets.Count} targets with tag {targetTag}");
    }

    private void UpdateCurrentTarget()
    {
        targets.RemoveAll(t => t == null);
        
        if (targets.Count > 0)
        {
            currentTarget = targets[0];
            Debug.Log($"Current target set to: {currentTarget.name}");
        }
        else
        {
            currentTarget = null;
            Debug.Log("No targets available");
        }
    }

    private void LateUpdate()
    {
        if (!hapticPlugin || currentTarget == null) return;

        UpdatePosition();
        
        // Maintain fixed rotation
        if (useFixedRotation)
        {
            transform.rotation = Quaternion.Euler(fixedRotation);
        }
    }

    private void UpdatePosition()
    {
        // Calculate world-space offset (independent of target's rotation)
        Vector3 worldOffset = new Vector3(
            offset.x, // Left/Right in world space
            offset.y, // Up/Down in world space
            offset.z  // Forward/Back in world space
        );

        // Calculate desired position by adding world-space offset to target position
        Vector3 desiredPosition = currentTarget.position + worldOffset;

        // Move the entire haptic device
        transform.position = Vector3.SmoothDamp(
            transform.position,
            desiredPosition,
            ref currentVelocity,
            1f / positionSmoothSpeed
        );
    }

    private void OnDrawGizmos()
    {
        if (!showDebugVisuals || currentTarget == null) return;

        // Draw line to current target
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, currentTarget.position);

        // Draw offset visualization
        Vector3 targetPos = currentTarget.position;
        
        // Draw world-space offset components
        Gizmos.color = Color.red;    // X axis offset
        Gizmos.DrawLine(targetPos, targetPos + new Vector3(offset.x, 0, 0));
        
        Gizmos.color = Color.green;  // Y axis offset
        Gizmos.DrawLine(targetPos + new Vector3(offset.x, 0, 0), 
                       targetPos + new Vector3(offset.x, offset.y, 0));
        
        Gizmos.color = Color.blue;   // Z axis offset
        Gizmos.DrawLine(targetPos + new Vector3(offset.x, offset.y, 0),
                       targetPos + new Vector3(offset.x, offset.y, offset.z));
    }

    // Public methods for managing targets
    public void AddTarget(Transform target)
    {
        if (target != null && !targets.Contains(target))
        {
            targets.Add(target);
            UpdateCurrentTarget();
        }
    }

    public void RemoveTarget(Transform target)
    {
        if (targets.Remove(target))
        {
            UpdateCurrentTarget();
        }
    }

    public void ClearTargets()
    {
        targets.Clear();
        currentTarget = null;
    }

    public void RefreshTargets()
    {
        FindTargets();
        UpdateCurrentTarget();
    }

    // Method to set the fixed rotation at runtime
    public void SetFixedRotation(Vector3 newRotation)
    {
        fixedRotation = newRotation;
        if (useFixedRotation)
        {
            transform.rotation = Quaternion.Euler(fixedRotation);
        }
    }
}