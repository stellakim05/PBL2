using UnityEngine;
using System.Collections.Generic;

public class RaspimouseForceCalculator : MonoBehaviour
{
    public ArticulationBody baseLink;
    public ArticulationBody forceSensorLink;
    public float reportThreshold = 0.05f;
    public float reportInterval = 0.5f;
    public float forceSmoothingTime = 0.1f; // Time over which to smooth force measurements

    private Vector3 lastVelocity;
    private Vector3 lastPosition;
    private float lastReportTime;
    private Vector3 accumulatedInertialForce;
    private Vector3 accumulatedContactForce;
    private int forceCount;
    private Dictionary<Collider, ContactData> contactData = new Dictionary<Collider, ContactData>();

    private class ContactData
    {
        public Vector3 force;
        public float lastContactTime;
    }

    void Start()
    {
        if (baseLink == null) baseLink = transform.Find("base_footprint/base_link").GetComponent<ArticulationBody>();
        if (forceSensorLink == null) forceSensorLink = transform.Find("base_footprint/base_link/front_force_sensor_link").GetComponent<ArticulationBody>();
        
        lastVelocity = baseLink.velocity;
        lastPosition = baseLink.transform.position;
        lastReportTime = Time.time;
    }

    void FixedUpdate()
    {
        CalculateInertialForce();
        UpdateContactForces();
        ReportForces();
    }

    void CalculateInertialForce()
    {
        Vector3 displacement = baseLink.transform.position - lastPosition;
        Vector3 velocity = displacement / Time.fixedDeltaTime;
        Vector3 acceleration = (velocity - lastVelocity) / Time.fixedDeltaTime;
        Vector3 inertialForce = baseLink.mass * acceleration;

        accumulatedInertialForce += inertialForce;
        forceCount++;

        lastVelocity = velocity;
        lastPosition = baseLink.transform.position;
    }

    void UpdateContactForces()
    {
        List<Collider> removeList = new List<Collider>();
        foreach (var kvp in contactData)
        {
            if (Time.time - kvp.Value.lastContactTime > forceSmoothingTime)
            {
                removeList.Add(kvp.Key);
            }
        }
        foreach (var collider in removeList)
        {
            contactData.Remove(collider);
        }
    }

    void ReportForces()
    {
        if (Time.time - lastReportTime > reportInterval && forceCount > 0)
        {
            Vector3 averageInertialForce = accumulatedInertialForce / forceCount;
            Vector3 averageContactForce = Vector3.zero;
            foreach (var kvp in contactData)
            {
                averageContactForce += kvp.Value.force;
            }
            if (contactData.Count > 0)
            {
                averageContactForce /= contactData.Count;
            }

            if (averageInertialForce.magnitude > reportThreshold)
            {
                Debug.Log($"Raspimouse inertial force: {averageInertialForce.magnitude:F4} N, Direction: {averageInertialForce.normalized}");
            }

            if (averageContactForce.magnitude > reportThreshold)
            {
                Debug.Log($"Raspimouse contact force: {averageContactForce.magnitude:F4} N, Direction: {averageContactForce.normalized}");
            }

            accumulatedInertialForce = Vector3.zero;
            forceCount = 0;
            lastReportTime = Time.time;
        }
    }

    void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("Cube"))
        {
            Vector3 totalImpulse = Vector3.zero;
            foreach (ContactPoint contact in collision.contacts)
            {
                totalImpulse += collision.impulse;
            }

            Vector3 force = totalImpulse / Time.fixedDeltaTime;
            
            if (!contactData.ContainsKey(collision.collider))
            {
                contactData[collision.collider] = new ContactData();
            }
            contactData[collision.collider].force = Vector3.Lerp(contactData[collision.collider].force, force, Time.fixedDeltaTime / forceSmoothingTime);
            contactData[collision.collider].lastContactTime = Time.time;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Cube"))
        {
            Debug.Log("Raspimouse started colliding with Cube");
        }
    }

    void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Cube"))
        {
            Debug.Log("Raspimouse stopped colliding with Cube");
            contactData.Remove(collision.collider);
        }
    }
}