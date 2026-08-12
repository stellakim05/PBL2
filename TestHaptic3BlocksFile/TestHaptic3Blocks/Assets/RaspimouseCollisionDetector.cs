using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class RaspimouseCollisionDetector : MonoBehaviour
{
    public ArticulationBody baseLink;
    private Vector3 lastPosition;

    void Start()
    {
        if (baseLink == null)
        {
            baseLink = GetComponentInChildren<ArticulationBody>();
        }
        lastPosition = baseLink.transform.position;
    }

    void FixedUpdate()
    {
        Vector3 displacement = baseLink.transform.position - lastPosition;
        float speed = displacement.magnitude / Time.fixedDeltaTime;
        
        if (speed > 0.01f)  // Threshold to avoid noise
        {
            Debug.Log($"Raspimouse speed: {speed} m/s");
        }

        lastPosition = baseLink.transform.position;
    }
}