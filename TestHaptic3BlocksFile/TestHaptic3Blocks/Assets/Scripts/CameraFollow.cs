using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] private Transform target;        // The object to follow
    [SerializeField] private float smoothSpeed = 5f;  // How smoothly the camera follows
    [SerializeField] private Vector3 offset;          // Offset from the target position

    void LateUpdate()
    {
        if (target == null)
            return;

        // Calculate the desired position
        Vector3 desiredPosition = target.position + offset;
        
        // Smoothly move the camera towards the desired position
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);
        transform.position = smoothedPosition;

        // Optional: Make the camera look at the target
        transform.LookAt(target);
    }
}