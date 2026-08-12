using UnityEngine;
using System.Collections.Generic;

public class ForceSensor : MonoBehaviour
{
    public float forceThreshold = 0.05f;
    public float reportInterval = 0.5f;
    public bool visualizeForce = true;
    public float forceSmoothingFactor = 0.1f;

    private float lastReportTime;
    private Vector3 currentForce;
    private Vector3 smoothedForce;
    private LineRenderer forceVisualizer;

    void Start()
    {
        if (visualizeForce)
        {
            SetupForceVisualizer();
        }
    }

    void SetupForceVisualizer()
    {
        forceVisualizer = gameObject.AddComponent<LineRenderer>();
        forceVisualizer.startWidth = 0.01f;
        forceVisualizer.endWidth = 0.01f;
        forceVisualizer.material = new Material(Shader.Find("Sprites/Default"));
        forceVisualizer.startColor = Color.red;
        forceVisualizer.endColor = Color.red;
    }

    void FixedUpdate()
    {
        // Smooth the force
        smoothedForce = Vector3.Lerp(smoothedForce, currentForce, forceSmoothingFactor);

        // Reset current force for the next fixed update
        currentForce = Vector3.zero;

        if (Time.time - lastReportTime >= reportInterval)
        {
            ReportForce();
        }

        if (visualizeForce)
        {
            VisualizeForce(smoothedForce);
        }
    }

    void OnCollisionStay(Collision collision)
    {
        // Impulse over time (Change in momentum)
        Vector3 impulse = collision.impulse;
        currentForce += impulse / Time.fixedDeltaTime;
    }

    void ReportForce()
    {
        if (smoothedForce.magnitude > forceThreshold)
        {
            Debug.Log($"Force Sensor Reading: Magnitude = {smoothedForce.magnitude:F4} N, Direction = {smoothedForce.normalized}");
        }

        lastReportTime = Time.time;
    }

    void VisualizeForce(Vector3 force)
    {
        if (forceVisualizer != null)
        {
            forceVisualizer.SetPosition(0, transform.position);
            forceVisualizer.SetPosition(1, transform.position + force.normalized * force.magnitude * 0.1f); // Scale for visibility
        }
    }

    public Vector3 GetCurrentForce()
    {
        return smoothedForce;
    }
}