using UnityEngine;

public class ForkLinkController : MonoBehaviour 
{
    public ArticulationBody forkArticulation;
    public GameObject collisionMesh;
    public float moveSpeed = 1.0f;
    public float maxLimit = 0.05f;  // Set to match URDF limit
    public float minLimit = 0f;     // Set to match URDF limit

    private Vector3 lastCollisionMeshPosition;
    private HapticPlugin hapticPlugin;

    void Start()
    {
        // Debug initial setup
        Debug.Log("ForkLinkController Start");
        
        if (forkArticulation == null)
        {
            Debug.LogError("Fork ArticulationBody not assigned!");
            return;
        }

        if (collisionMesh == null)
        {
            Debug.LogError("Collision Mesh (Stylus tip) not assigned!");
            return;
        }

        lastCollisionMeshPosition = collisionMesh.transform.position;
        Debug.Log($"Initial collision mesh position: {lastCollisionMeshPosition}");
        Debug.Log($"Initial fork position: {forkArticulation.jointPosition[0]}");

        // Log the drive settings
        var drive = forkArticulation.zDrive;
        Debug.Log($"Drive settings - Lower Limit: {drive.lowerLimit}, Upper Limit: {drive.upperLimit}, Current Target: {drive.target}");
    }

    void FixedUpdate()
    {
        if (collisionMesh == null || forkArticulation == null) return;

        // Log current positions
        Vector3 currentCollisionMeshPos = collisionMesh.transform.position;
        float verticalDelta = currentCollisionMeshPos.y - lastCollisionMeshPosition.y;
        
        Debug.Log($"Current collision mesh Y: {currentCollisionMeshPos.y}");
        Debug.Log($"Last collision mesh Y: {lastCollisionMeshPosition.y}");
        Debug.Log($"Vertical delta: {verticalDelta}");

        // Get current fork position
        float currentPosition = forkArticulation.jointPosition[0];
        Debug.Log($"Current fork position: {currentPosition}");

        // Calculate new position
        float targetPosition = Mathf.Clamp(currentPosition + verticalDelta * moveSpeed, minLimit, maxLimit);
        Debug.Log($"Target position after clamp: {targetPosition}");

        // Apply new position to fork's Z-Drive
        ArticulationDrive drive = forkArticulation.zDrive;
        float oldTarget = drive.target;
        drive.target = targetPosition;
        forkArticulation.zDrive = drive;
        Debug.Log($"Drive target changed from {oldTarget} to {drive.target}");

        // Store current position for next frame
        lastCollisionMeshPosition = currentCollisionMeshPos;
    }

    // OnEnable to verify component references
    void OnEnable()
    {
        Debug.Log("ForkLinkController enabled");
        if (forkArticulation != null)
        {
            Debug.Log($"Fork ArticulationBody initial state - Position: {forkArticulation.jointPosition[0]}");
            var drive = forkArticulation.zDrive;
            Debug.Log($"Z Drive settings - Target: {drive.target}, Limits: [{drive.lowerLimit}, {drive.upperLimit}]");
        }
    }
}