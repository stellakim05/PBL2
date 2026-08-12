using UnityEngine;

[RequireComponent(typeof(HapticPlugin))]
public class HapticInteractionToggler : MonoBehaviour
{
    [Header("Connection Settings")]
    [Tooltip("Key to toggle haptic connection")]
    public KeyCode toggleKey = KeyCode.H;
    
    [Tooltip("Optional key to manually disconnect")]
    public KeyCode disconnectKey = KeyCode.J;

    [Header("Auto-Connection")]
    [Tooltip("Enable haptic device automatically when script is enabled")]
    public bool connectOnEnable = false;
    
    [Header("Status")]
    [SerializeField]
    private bool isConnected = false;
    
    // References
    private HapticPlugin hapticPlugin;
    private Rigidbody collisionRigidbody;
    private bool originalKinematicState;
    private CollisionDetectionMode originalCollisionMode;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    void Awake()
    {
        // Get references early
        hapticPlugin = GetComponent<HapticPlugin>();
        if (hapticPlugin == null)
        {
            Debug.LogError("HapticPlugin component not found!");
            enabled = false;
            return;
        }

        // Make sure HapticPlugin doesn't auto-connect
        hapticPlugin.ConnectOnStart = false;
    }

    void OnEnable()
    {
        InitializeComponents();
        
        if (connectOnEnable)
        {
            ConnectHaptic();
        }
        else
        {
            SetCollisionMeshState(false);
        }
    }

    private void InitializeComponents()
    {
        // Store initial transforms
        if (hapticPlugin.CollisionMesh != null)
        {
            initialPosition = hapticPlugin.CollisionMesh.transform.position;
            initialRotation = hapticPlugin.CollisionMesh.transform.rotation;
            
            collisionRigidbody = hapticPlugin.CollisionMesh.GetComponent<Rigidbody>();
            if (collisionRigidbody != null)
            {
                originalKinematicState = collisionRigidbody.isKinematic;
                originalCollisionMode = collisionRigidbody.collisionDetectionMode;
            }
        }

        if (hapticPlugin.VisualizationMesh != null)
        {
            // Ensure visualization mesh has valid initial transform
            if (hapticPlugin.VisualizationMesh.transform.forward == Vector3.zero)
            {
                hapticPlugin.VisualizationMesh.transform.rotation = Quaternion.identity;
            }
        }
    }

    void Update()
    {
        if (!enabled) return;

        // Toggle connection with specified key
        if (Input.GetKeyDown(toggleKey))
        {
            ToggleConnection();
        }

        // Manual disconnect with specified key
        if (Input.GetKeyDown(disconnectKey))
        {
            DisconnectHaptic();
        }
    }

    private void SetCollisionMeshState(bool connected)
    {
        if (collisionRigidbody == null || hapticPlugin.CollisionMesh == null) return;

        if (connected)
        {
            // When connected, use original settings
            collisionRigidbody.isKinematic = originalKinematicState;
            collisionRigidbody.collisionDetectionMode = originalCollisionMode;
            collisionRigidbody.useGravity = false;
            collisionRigidbody.interpolation = RigidbodyInterpolation.None;
        }
        else
        {
            // When disconnected, use safe settings
            collisionRigidbody.isKinematic = true;
            collisionRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            collisionRigidbody.useGravity = false;
            collisionRigidbody.interpolation = RigidbodyInterpolation.None;

            // Reset transforms to initial state
            hapticPlugin.CollisionMesh.transform.position = initialPosition;
            hapticPlugin.CollisionMesh.transform.rotation = initialRotation;
            if (hapticPlugin.VisualizationMesh != null)
            {
                hapticPlugin.VisualizationMesh.transform.position = initialPosition;
                hapticPlugin.VisualizationMesh.transform.rotation = initialRotation;
            }
        }
    }

    public void ToggleConnection()
    {
        if (!isConnected)
        {
            ConnectHaptic();
        }
        else
        {
            DisconnectHaptic();
        }
    }

    public void ConnectHaptic()
    {
        if (!isConnected && hapticPlugin != null)
        {
            if (hapticPlugin.InitializeHapticDevice())
            {
                isConnected = true;
                SetCollisionMeshState(true);
                Debug.Log("Haptic device connected successfully.");
            }
            else
            {
                Debug.LogError("Failed to connect to haptic device!");
                SetCollisionMeshState(false);
            }
        }
    }

    public void DisconnectHaptic()
    {
        if (isConnected && hapticPlugin != null)
        {
            hapticPlugin.DisconnectHapticDevice();
            isConnected = false;
            SetCollisionMeshState(false);
            Debug.Log("Haptic device disconnected.");
        }
    }

    void OnDisable()
    {
        if (enabled)
        {
            DisconnectHaptic();
        }
    }

    void OnApplicationQuit()
    {
        DisconnectHaptic();
    }

    public bool IsConnected()
    {
        return isConnected;
    }

    public void ForceEnable()
    {
        enabled = true;
        InitializeComponents();
        if (connectOnEnable)
        {
            ConnectHaptic();
        }
    }
}