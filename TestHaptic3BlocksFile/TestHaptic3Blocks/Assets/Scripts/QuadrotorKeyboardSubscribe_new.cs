using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class QuadrotorKeyboardSubscribe_new : MonoBehaviour
{
    [Header("Rotor Configuration")]
    public ArticulationBody baseLink;
    public ArticulationBody rotorFrontRight;
    public ArticulationBody rotorFrontLeft;
    public ArticulationBody rotorRearRight;
    public ArticulationBody rotorRearLeft;
    
    [Header("Flight Parameters")]
    public float maxThrust = 15.0f;
    public float mass = 1.5f;
    public float dragCoefficient = 0.1f;
    public float maxTiltAngle = 30.0f;
    public float rotorForceMultiplier = 0.02f;
    
    [Header("Movement Settings")]
    public float maxHorizontalSpeed = 5.0f;
    public float maxVerticalSpeed = 3.0f;
    public float maxAngularSpeed = 2.0f;
    public float movementForceMultiplier = 1.0f;
    public float heightChangeRate = 1.0f;
    
    [Header("Stability")]
    public float stabilityForce = 15.0f;
    public float angularDamping = 2.0f;
    public float linearDamping = 0.5f;
    public float heightControlP = 5.0f;
    public float heightControlD = 2.0f;
    public float tiltCompensation = 2.0f;

    private Dictionary<string, float> rotorSpeeds = new Dictionary<string, float>();
    private Vector3 targetVelocity;
    private float targetYawRate;
    private bool isFlying = false;
    private float targetHeight;
    private Vector3 initialPosition;
    private float verticalInput = 0f;
    private Vector3 lastVelocity = Vector3.zero;
    private float lastHeightError = 0f;
    private const float velocitySmoothTime = 0.1f;
    
    private ROSConnection ros;

    private void Start()
    {
        InitializeComponents();
    }

    private void InitializeComponents()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", UpdateMovement);
        ros.Subscribe<EmptyMsg>("drone/takeoff", HandleTakeoff);
        ros.Subscribe<EmptyMsg>("drone/land", HandleLand);
        ros.Subscribe<Float64Msg>("rotor_front_right_joint/command", speed => UpdateRotorSpeed("front_right", speed.data));
        ros.Subscribe<Float64Msg>("rotor_front_left_joint/command", speed => UpdateRotorSpeed("front_left", speed.data));
        ros.Subscribe<Float64Msg>("rotor_rear_right_joint/command", speed => UpdateRotorSpeed("rear_right", speed.data));
        ros.Subscribe<Float64Msg>("rotor_rear_left_joint/command", speed => UpdateRotorSpeed("rear_left", speed.data));

        InitializeRotorSpeeds();
        ConfigureBaseLink();
    }

    private void InitializeRotorSpeeds()
    {
        rotorSpeeds["front_right"] = 0f;
        rotorSpeeds["front_left"] = 0f;
        rotorSpeeds["rear_right"] = 0f;
        rotorSpeeds["rear_left"] = 0f;
    }

    private void ConfigureBaseLink()
    {
        if (!baseLink) { Debug.LogError("Base link missing!"); return; }
        
        baseLink.mass = mass;
        baseLink.angularDamping = angularDamping;
        baseLink.linearDamping = linearDamping;
        initialPosition = baseLink.transform.position;
        targetHeight = initialPosition.y;
    }

    private void UpdateMovement(TwistMsg message)
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
        }
    }

    private void UpdateRotorSpeed(string rotorName, double speed)
    {
        rotorSpeeds[rotorName] = (float)speed * rotorForceMultiplier;
    }

    private void FixedUpdate()
    {
        if (!isFlying || !baseLink) return;

        ApplyRotorForces();
        ApplyStabilization();
        ApplyMovement();
        ApplyHoverForce();
        ApplyRotation();
    }

    private void ApplyRotorForces()
    {
        if (rotorFrontRight) ApplyRotorForce(rotorFrontRight, rotorSpeeds["front_right"]);
        if (rotorFrontLeft) ApplyRotorForce(rotorFrontLeft, rotorSpeeds["front_left"]);
        if (rotorRearRight) ApplyRotorForce(rotorRearRight, rotorSpeeds["rear_right"]);
        if (rotorRearLeft) ApplyRotorForce(rotorRearLeft, rotorSpeeds["rear_left"]);
    }

    private void ApplyRotorForce(ArticulationBody rotor, float speed)
    {
        Vector3 force = rotor.transform.up * speed;
        rotor.AddForce(force);
        baseLink.AddTorque(-rotor.transform.up * speed * 0.05f);
    }

    private void ApplyStabilization()
    {
        Vector3 currentUp = baseLink.transform.up;
        Vector3 desiredUp = Vector3.up;
        Vector3 stabilizationTorque = Vector3.Cross(currentUp, desiredUp) * stabilityForce;
        baseLink.AddTorque(stabilizationTorque);
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
        float verticalForce = verticalInput * maxVerticalSpeed * movementForceMultiplier * 0.5f;
        
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

    private void HandleTakeoff(EmptyMsg msg)
    {
        isFlying = true;
        targetHeight = baseLink.transform.position.y + 2.0f;
    }

    private void HandleLand(EmptyMsg msg)
    {
        isFlying = false;
        targetHeight = initialPosition.y;
    }

    private void OnGUI()
    {
        if (!baseLink) return;
        
        GUI.color = new Color(0, 0, 0, 0.7f);
        GUI.Box(new Rect(0, 30, 300, 200), "");
        
        GUI.color = Color.white;
        GUILayout.BeginArea(new Rect(0, 30, 290, 190));
        var style = new GUIStyle(GUI.skin.label) { normal = { textColor = Color.white } };
        
        GUILayout.Label($"Flying: {isFlying}", style);
        GUILayout.Label($"Height: {baseLink.transform.position.y:F2}m", style);
        GUILayout.Label($"Target Height: {targetHeight:F2}m", style);
        GUILayout.Label($"Velocity: {baseLink.velocity.magnitude:F2} m/s", style);
        GUILayout.Label($"Vertical Input: {verticalInput:F2}", style);
        GUILayout.Label($"Front Right Speed: {rotorSpeeds["front_right"]:F2}", style);
        GUILayout.Label($"Front Left Speed: {rotorSpeeds["front_left"]:F2}", style);
        GUILayout.Label($"Rear Right Speed: {rotorSpeeds["rear_right"]:F2}", style);
        GUILayout.Label($"Rear Left Speed: {rotorSpeeds["rear_left"]:F2}", style);
        
        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || !baseLink) return;

        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, targetVelocity);

        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, baseLink.velocity);

        Gizmos.color = Color.yellow;
        Vector3 targetPos = transform.position;
        targetPos.y = targetHeight;
        Gizmos.DrawWireSphere(targetPos, 0.5f);

        Gizmos.color = Color.green;
        if (rotorFrontRight)
            Gizmos.DrawRay(rotorFrontRight.transform.position, rotorFrontRight.transform.up * rotorSpeeds["front_right"]);
        if (rotorFrontLeft)
            Gizmos.DrawRay(rotorFrontLeft.transform.position, rotorFrontLeft.transform.up * rotorSpeeds["front_left"]);
        if (rotorRearRight)
            Gizmos.DrawRay(rotorRearRight.transform.position, rotorRearRight.transform.up * rotorSpeeds["rear_right"]);
        if (rotorRearLeft)
            Gizmos.DrawRay(rotorRearLeft.transform.position, rotorRearLeft.transform.up * rotorSpeeds["rear_left"]);
    }
}