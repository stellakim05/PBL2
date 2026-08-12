using UnityEngine;

public class ExperimentSetup : MonoBehaviour 
{
    [Header("Scene Objects")]
    public GameObject robot1;    // Left robot
    public GameObject robot2;    // Right robot
    public GameObject box;
    
    [Header("Setup Parameters")]
    public float distanceBetweenRobots = 1.4f;
    public float boxRotationAngle = 0f;
    public float robotOffsetAngle = 5f;
    public float boxForwardOffset = 0.2f;
    
    [Header("Movement Constraints")]
    public bool lockRobotYPosition = true;     // Keep robots at same Y level
    public bool maintainMinDistance = true;    // Prevent robots from getting too close
    public float minimumRobotDistance = 1.0f;  // Minimum allowed distance between robots
    
    [Header("Robot Movement")]
    public float maxPushForce = 5f;           // Maximum force robots can apply
    public float pushDirectionTolerance = 0.1f; // How much deviation from ideal push direction is allowed
    
    private Rigidbody robot1Rb;
    private Rigidbody robot2Rb;
    private Rigidbody boxRb;

    void Start()
    {
        InitializeComponents();
        CenterObjects();
    }

    void InitializeComponents()
    {
        robot1Rb = robot1.GetComponent<Rigidbody>();
        robot2Rb = robot2.GetComponent<Rigidbody>();
        boxRb = box.GetComponent<Rigidbody>();
        
        // Configure rigidbodies
        if (robot1Rb && robot2Rb)
        {
            ConfigureRobotRigidbody(robot1Rb);
            ConfigureRobotRigidbody(robot2Rb);
        }
    }

    void ConfigureRobotRigidbody(Rigidbody rb)
    {
        rb.constraints = RigidbodyConstraints.FreezeRotationX | 
                        RigidbodyConstraints.FreezeRotationZ;
        if (lockRobotYPosition)
        {
            rb.constraints |= RigidbodyConstraints.FreezePositionY;
        }
    }

    void FixedUpdate()
    {
        if (maintainMinDistance)
        {
            EnforceMinimumDistance();
        }
        
        CorrectPushDirection();
    }

    void EnforceMinimumDistance()
    {
        float currentDistance = Vector3.Distance(robot1.transform.position, robot2.transform.position);
        if (currentDistance < minimumRobotDistance)
        {
            // Calculate correction vector
            Vector3 direction = (robot2.transform.position - robot1.transform.position).normalized;
            float correction = (minimumRobotDistance - currentDistance) / 2f;
            
            // Apply correction
            robot1.transform.position -= direction * correction;
            robot2.transform.position += direction * correction;
        }
    }

    void CorrectPushDirection()
    {
        if (robot1Rb && robot2Rb && boxRb)
        {
            // Get ideal push directions (perpendicular to box faces)
            Vector3 idealDirection1 = (box.transform.position - robot1.transform.position).normalized;
            Vector3 idealDirection2 = (box.transform.position - robot2.transform.position).normalized;
            
            // Get current velocity directions
            Vector3 currentVel1 = robot1Rb.velocity.normalized;
            Vector3 currentVel2 = robot2Rb.velocity.normalized;
            
            // If deviation is too large, correct it
            if (Vector3.Dot(currentVel1, idealDirection1) < (1 - pushDirectionTolerance))
            {
                robot1Rb.velocity = idealDirection1 * robot1Rb.velocity.magnitude;
            }
            
            if (Vector3.Dot(currentVel2, idealDirection2) < (1 - pushDirectionTolerance))
            {
                robot2Rb.velocity = idealDirection2 * robot2Rb.velocity.magnitude;
            }
        }
    }

    public void ApplyPushForces(float leftForce, float rightForce)
    {
        if (robot1Rb && robot2Rb && boxRb)
        {
            // Calculate push directions
            Vector3 pushDir1 = (box.transform.position - robot1.transform.position).normalized;
            Vector3 pushDir2 = (box.transform.position - robot2.transform.position).normalized;
            
            // Apply forces with magnitude limits
            float clampedLeftForce = Mathf.Clamp(leftForce, 0, maxPushForce);
            float clampedRightForce = Mathf.Clamp(rightForce, 0, maxPushForce);
            
            robot1Rb.AddForce(pushDir1 * clampedLeftForce, ForceMode.Force);
            robot2Rb.AddForce(pushDir2 * clampedRightForce, ForceMode.Force);
        }
    }
    public void CenterObjects()
    {
        // Ensure we have all required objects
        if (robot1 == null || robot2 == null || box == null)
        {
            Debug.LogError("Missing required objects! Please assign all objects in inspector.");
            return;
        }

        // Calculate positions
        Vector3 centerPosition = Vector3.zero;
        float halfDistance = distanceBetweenRobots / 2f;

        // Position robots
        robot1.transform.position = centerPosition + new Vector3(-halfDistance, 0, 0);
        robot2.transform.position = centerPosition + new Vector3(halfDistance, 0, 0);

        // Center box and offset it forward
        Vector3 boxPosition = centerPosition + new Vector3(0, 0, boxForwardOffset);
        box.transform.position = boxPosition;
        
        // Apply initial rotation to box
        box.transform.rotation = Quaternion.Euler(0, boxRotationAngle, 0);

        // Apply offset angles to robots if needed
        if (robotOffsetAngle != 0)
        {
            robot1.transform.rotation = Quaternion.Euler(0, -robotOffsetAngle, 0);
            robot2.transform.rotation = Quaternion.Euler(0, robotOffsetAngle, 0);
        }

        // Optional: Add validation checks
        ValidateSetup();
    }

    void ValidateSetup()
    {
        // Check actual distances
        float actualDistance = Vector3.Distance(robot1.transform.position, robot2.transform.position);
        if (Mathf.Abs(actualDistance - distanceBetweenRobots) > 0.01f)
        {
            Debug.LogWarning($"Distance validation failed. Expected: {distanceBetweenRobots}, Actual: {actualDistance}");
        }

        // Check if box is centered between robots (ignoring Z-offset)
        Vector3 robot1XYPos = new Vector3(robot1.transform.position.x, 0, 0);
        Vector3 robot2XYPos = new Vector3(robot2.transform.position.x, 0, 0);
        Vector3 boxXYPos = new Vector3(box.transform.position.x, 0, 0);
        
        float distanceToRobot1 = Vector3.Distance(boxXYPos, robot1XYPos);
        float distanceToRobot2 = Vector3.Distance(boxXYPos, robot2XYPos);
        
        if (Mathf.Abs(distanceToRobot1 - distanceToRobot2) > 0.01f)
        {
            Debug.LogWarning($"Box is not perfectly centered! Difference: {Mathf.Abs(distanceToRobot1 - distanceToRobot2)}m");
        }

        // Validate Z-offset
        if (Mathf.Abs(box.transform.position.z - boxForwardOffset) > 0.01f)
        {
            Debug.LogWarning($"Box Z-offset incorrect. Expected: {boxForwardOffset}, Actual: {box.transform.position.z}");
        }
    }

    // Editor button to recenter objects
    [ContextMenu("Recenter Objects")]
    public void EditorRecenter()
    {
        CenterObjects();
    }
}