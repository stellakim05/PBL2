using UnityEngine;

public class CubeCollisionDetector : MonoBehaviour
{
    public float reportThreshold = 0.05f; // Increased threshold
    public float reportInterval = 0.5f; // Report every 0.5 seconds

    private Rigidbody rb;
    private Vector3 lastVelocity;
    private float lastReportTime;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        lastVelocity = rb.velocity;
        lastReportTime = Time.time;
    }

    void FixedUpdate()
    {
        Vector3 acceleration = (rb.velocity - lastVelocity) / Time.fixedDeltaTime;
        Vector3 force = rb.mass * acceleration;

        if (force.magnitude > reportThreshold && Time.time - lastReportTime > reportInterval)
        {
            Debug.Log($"Force on cube: {force.magnitude:F4} N, Direction: {force.normalized}");
            lastReportTime = Time.time;
        }

        lastVelocity = rb.velocity;
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Raspimouse"))
        {
            Debug.Log("Cube started colliding with Raspimouse");
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Raspimouse"))
        {
            Debug.Log("Cube stopped colliding with Raspimouse");
        }
    }
}