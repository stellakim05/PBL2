using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class QuadrotorKeyboardSubscriber : MonoBehaviour
{
    [Header("Flight Characteristics")]
    public float maxThrust = 15.0f;
    public float mass = 1.5f;
    public float dragCoefficient = 0.1f;
    public float maxTiltAngle = 30.0f;
    
    [Header("Movement Settings")]
    public float maxHorizontalSpeed = 5.0f;
    public float maxVerticalSpeed = 3.0f;
    public float maxAngularSpeed = 2.0f;
    public float movementForceMultiplier = 1.0f;
    public float heightChangeRate = 1.0f;
    
    [Header("Stability Settings")]
    public float stabilityForce = 10.0f;
    public float heightControlP = 5.0f;
    public float heightControlD = 2.0f;
    public float tiltCompensation = 2.0f;
    
    [Header("Realistic Movement")]
    public float windStrength = 0.1f;
    public float windChangeSpeed = 0.1f;
    public float motorVibrationStrength = 0.1f;
    public float turbulenceStrength =0.1f;
    public float driftStrength = 0.1f;
    
    private ROSConnection ros;
    public ArticulationBody baseLink;
    
    // State variables
    private Vector3 targetVelocity;
    private float targetYawRate;
    private bool isFlying = false;
    private float targetHeight;
    private Vector3 initialPosition;
    private Quaternion targetRotation = Quaternion.identity;
    private float verticalInput = 0f;
    
    // Stability variables
    private float lastHeightError = 0f;
    private Vector3 lastVelocity = Vector3.zero;
    private float smoothDampVelocity = 0f;
    private const float velocitySmoothTime = 0.1f;
    
    // Noise variables
    private Vector3 currentWindForce;
    private Vector3 targetWindForce;
    private float windUpdateTime;
    private Vector3[] turbulenceOffsets;
    private float[] turbulencePhases;
    private Vector3 driftOffset;
    private float timeOffset;

    private void Start()
    {
        InitializeComponents();
        InitializeNoiseVariables();
    }
    
    private void InitializeComponents()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", UpdateQuadrotorVelocity);
        ros.Subscribe<EmptyMsg>("drone/takeoff", HandleTakeoff);
        ros.Subscribe<EmptyMsg>("drone/land", HandleLand);
        
        if (baseLink == null)
        {
            Debug.LogError("ArticulationBody component is not assigned!");
            return;
        }

        baseLink.mass = mass;
        baseLink.angularDamping = 0.8f;
        baseLink.linearDamping = 0.5f;
        
        initialPosition = baseLink.transform.position;
        targetHeight = initialPosition.y;
        
        Debug.Log("QuadrotorKeyboardSubscriber initialized");
    }
    
    private void InitializeNoiseVariables()
    {
        turbulenceOffsets = new Vector3[3];
        turbulencePhases = new float[3];
        for (int i = 0; i < 3; i++)
        {
            turbulenceOffsets[i] = Random.onUnitSphere;
            turbulencePhases[i] = Random.value * Mathf.PI * 2;
        }
        
        currentWindForce = Random.onUnitSphere * windStrength;
        targetWindForce = Random.onUnitSphere * windStrength;
        windUpdateTime = Time.time;
        timeOffset = Random.value * 1000f;
    }

    private void UpdateQuadrotorVelocity(TwistMsg message)
    {
        if (!isFlying) return;
        
        verticalInput = (float)message.linear.z;
        
        Vector3 newTargetVelocity = new Vector3(
            (float)message.linear.x * maxHorizontalSpeed,
            0f,
            -(float)message.linear.y * maxHorizontalSpeed
        );
        
        targetVelocity = Vector3.SmoothDamp(
            targetVelocity, 
            newTargetVelocity, 
            ref lastVelocity, 
            velocitySmoothTime
        );
        
        targetYawRate = (float)message.angular.z * maxAngularSpeed;
        
        if (Mathf.Abs(verticalInput) > 0.01f)
        {
            targetHeight += verticalInput * heightChangeRate * Time.fixedDeltaTime;
            Debug.Log($"Updating target height: {targetHeight} (input: {verticalInput})");
        }
    }

    private void FixedUpdate()
    {
        if (!isFlying || baseLink == null) return;

        UpdateNoise();
        ApplyStabilizingForces();
        ApplyMovement();
        ApplyHoverForce();
        ApplyRotation();
        ApplyRealisticForces();
        
    }
    
    private void UpdateNoise()
    {
        // Update wind
        if (Time.time - windUpdateTime > 1f / windChangeSpeed)
        {
            targetWindForce = Random.onUnitSphere * windStrength;
            windUpdateTime = Time.time;
        }
        currentWindForce = Vector3.Lerp(currentWindForce, targetWindForce, Time.deltaTime * windChangeSpeed);
        
        // Update drift
        float t = (Time.time + timeOffset) * 0.5f;
        driftOffset = new Vector3(
            Mathf.PerlinNoise(t, 0) - 0.5f,
            Mathf.PerlinNoise(0, t) - 0.5f,
            Mathf.PerlinNoise(t, t) - 0.5f
        ) * driftStrength;
    }

    private void ApplyStabilizingForces()
    {
        Vector3 currentUp = baseLink.transform.up;
        Vector3 desiredUp = Vector3.up;
        Vector3 stabilizationTorque = Vector3.Cross(currentUp, desiredUp) * stabilityForce;
        baseLink.AddTorque(stabilizationTorque);
        
        Vector3 angularVelocityDamping = -baseLink.angularVelocity * 2.0f;
        baseLink.AddTorque(angularVelocityDamping);
    }

    private void ApplyMovement()
    {
        Vector3 movementForce = targetVelocity * movementForceMultiplier;
        movementForce = baseLink.transform.TransformDirection(movementForce);
        
        float tiltAngle = Vector3.Angle(baseLink.transform.up, Vector3.up);
        Vector3 tiltCompensationForce = Vector3.up * (tiltAngle * tiltCompensation);
        
        baseLink.AddForce(movementForce + tiltCompensationForce);
        
        Vector3 drag = -baseLink.velocity.normalized * (baseLink.velocity.sqrMagnitude * dragCoefficient);
        baseLink.AddForce(drag);
    }

    private void ApplyHoverForce()
    {
        float heightError = targetHeight - baseLink.transform.position.y;
        float heightErrorDerivative = (heightError - lastHeightError) / Time.fixedDeltaTime;
        lastHeightError = heightError;
        
        float heightControl = heightError * heightControlP + heightErrorDerivative * heightControlD;
        
        float gravityCompensation = -Physics.gravity.y * mass;
        float verticalForce = verticalInput * maxVerticalSpeed * movementForceMultiplier;
        Vector3 totalVerticalForce = Vector3.up * (gravityCompensation + heightControl + verticalForce);
        
        baseLink.AddForce(totalVerticalForce);
    }

    private void ApplyRotation()
    {
        if (Mathf.Abs(targetYawRate) > 0.01f)
        {
            Vector3 torque = Vector3.up * targetYawRate * movementForceMultiplier;
            baseLink.AddTorque(torque);
        }
    }

    private void ApplyRealisticForces()
    {
        // Apply wind force
        baseLink.AddForce(currentWindForce);
        
        // Apply turbulence
        Vector3 turbulence = Vector3.zero;
        float time = Time.time + timeOffset;
        for (int i = 0; i < 3; i++)
        {
            turbulence += turbulenceOffsets[i] * Mathf.Sin(time * (1f + i * 0.5f) + turbulencePhases[i]);
        }
        baseLink.AddForce(turbulence * turbulenceStrength);
        
        // Apply motor vibrations
        Vector3 vibration = new Vector3(
            Mathf.Sin(time * 50f),
            Mathf.Sin(time * 51f),
            Mathf.Sin(time * 52f)
        ) * motorVibrationStrength;
        baseLink.AddForce(vibration);
        
        // Apply drift
        baseLink.AddForce(driftOffset);
    }

    private void HandleTakeoff(EmptyMsg msg)
    {
        isFlying = true;
        targetHeight = baseLink.transform.position.y ;
        Debug.Log("Takeoff command received - Target Height: " + targetHeight);
    }
    
    private void HandleLand(EmptyMsg msg)
    {
        isFlying = false;
        targetHeight = initialPosition.y;
        Debug.Log("Land command received");
    }
    
    private void OnGUI()
    {
        if (baseLink != null)
        {   // Draw background box
            GUI.color = new Color(0, 0, 0, 0.7f);
            GUI.Box(new Rect(0, 30, 300, 200), "");
            GUI.color = Color.white;
            GUILayout.BeginArea(new Rect(0, 30, 290, 190));
            var style = new GUIStyle(GUI.skin.label);
            style.normal.textColor = Color.white;
            GUILayout.Label($"Flying: {isFlying}", style);
            GUILayout.Label($"Height: {baseLink.transform.position.y:F2}m", style);
            GUILayout.Label($"Target Height: {targetHeight:F2}m", style);
            GUILayout.Label($"Vertical Input: {verticalInput:F2}", style);
            GUILayout.Label($"Current Velocity: {baseLink.velocity:F2}", style);
            GUILayout.Label($"Wind Force: {currentWindForce.magnitude:F2}", style);
            GUILayout.EndArea();
        }
    }

    // private void OnDrawGizmos()
    // {
    //     if (baseLink == null || !Application.isPlaying) return;

    //     Gizmos.color = Color.blue;
    //     Gizmos.DrawRay(transform.position, targetVelocity);

    //     Gizmos.color = Color.red;
    //     Gizmos.DrawRay(transform.position, baseLink.velocity);

    //     Gizmos.color = Color.yellow;
    //     Vector3 targetPos = transform.position;
    //     targetPos.y = targetHeight;
    //     Gizmos.DrawWireSphere(targetPos, 0.5f);
        
    //     // Draw wind force
    //     Gizmos.color = Color.cyan;
    //     Gizmos.DrawRay(transform.position, currentWindForce);
    // }
   
}