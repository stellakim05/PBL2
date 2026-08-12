using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;

[Serializable]
public class HapticContact
{
    public float startTime;
    public float endTime;
    public float duration;
    public float maxForce;
    public float averageForce;
    public Vector3 contactPoint;
    public string objectType;  // "box", "robot1", "robot2"
    public List<float> forceValues;

    public HapticContact()
    {
        forceValues = new List<float>();
    }

    public void Initialize(float time, Vector3 point, string type)
    {
        startTime = time;
        contactPoint = point;
        objectType = type;
        maxForce = 0f;
        averageForce = 0f;
        forceValues = new List<float>();
    }
}

public class ExperimentDataCollector : MonoBehaviour
{
    [Header("Objects References")]
    public GameObject box;
    public GameObject robot1;
    public GameObject robot2;
    public HapticPlugin hapticDevice;

    [Header("Recording Settings")]
    public float samplingRate = 0.02f; // 50Hz
    public float forceSamplingRate = 0.016f; // 60Hz for force sampling
    public bool autoStartRecording = false;

    [Header("Experimental Parameters")]
    public float targetRotation = 0f;
    public float rotationThreshold = 5f;
    public float contactForceThreshold = 0.01f;

    [Header("Debug")]
    public bool showTimeDebug = true;
    
    // Timing variables
    private float experimentStartTime;
    private float currentTaskTime;
    private bool isRecording = false;
    private float lastSampleTime;
    private float lastForceSampleTime;
    
    // Session info
    private string sessionID;
    private string dataPath;
    private StreamWriter writer;
    private StreamWriter contactWriter;

    // Performance metrics
    private float totalRotationError = 0f;
    private int rotationErrorSamples = 0;
    private float maxRotationDeviation = 0f;
    private int stabilityViolations = 0;
    private float cumulativeContactTime = 0f;

    // Contact tracking
    private HapticContact currentContact = null;
    private List<HapticContact> contactHistory = new List<HapticContact>();
    private bool isInContact = false;
    private string currentContactType = "";

    void Start()
    {
        InitializeSession();
        if (autoStartRecording)
        {
            StartRecording();
        }
    }

    void InitializeSession()
    {
        sessionID = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        dataPath = Path.Combine(Application.dataPath, "ExperimentData");
        Directory.CreateDirectory(dataPath);
        currentTaskTime = 0f;
    }

    public void StartRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("Already recording!");
            return;
        }

        experimentStartTime = Time.time;
        lastSampleTime = Time.time;
        lastForceSampleTime = Time.time;
        isRecording = true;
        currentTaskTime = 0f;
        
        string filename = Path.Combine(dataPath, $"experiment_session_{sessionID}.csv");
        writer = new StreamWriter(filename, false);
        
        string contactFilename = Path.Combine(dataPath, $"contact_data_{sessionID}.csv");
        contactWriter = new StreamWriter(contactFilename, false);
        
        WriteHeaders();
        
        Debug.Log($"Started recording session: {sessionID} at time: {experimentStartTime}");
    }

    void WriteHeaders()
    {
        // Main experiment data header
        writer.WriteLine(
            "TaskTime,BoxRotation,RotationError,BoxPosX,BoxPosY,BoxPosZ," +
            "BoxAngVelX,BoxAngVelY,BoxAngVelZ," +
            "Robot1PosX,Robot1PosY,Robot1PosZ,Robot2PosX,Robot2PosY,Robot2PosZ," +
            "Robot1Speed,Robot2Speed,RobotDistanceDiff," +
            "HapticPosX,HapticPosY,HapticPosZ," +
            "HapticForceX,HapticForceY,HapticForceZ,ForceMagnitude," +
            "IsInContact,ContactType,ContactDuration,Phase," +
            "CumulativeError,StabilityMetric"
        );

        // Contact data header
        contactWriter.WriteLine(
            "ContactID,ObjectType,StartTime,EndTime,Duration,MaxForce,AverageForce," +
            "ContactPointX,ContactPointY,ContactPointZ"
        );
    }

    void Update()
    {
        if (!isRecording) return;

        currentTaskTime = Time.time - experimentStartTime;

        if (showTimeDebug)
        {
            Debug.Log($"Task Time: {currentTaskTime:F2} seconds");
        }

        // Check for contacts and record force data
        CheckAndUpdateContacts();

        // Record data at specified sampling rate
        if (Time.time - lastSampleTime >= samplingRate)
        {
            RecordDataPoint();
            lastSampleTime = Time.time;
        }
    }

    void CheckAndUpdateContacts()
    {
        bool newContact = false;
        string objectType = "";
        Vector3 contactPoint = Vector3.zero;

        if (hapticDevice.MagForce > contactForceThreshold)
        {
            Collider[] colliders = Physics.OverlapSphere(hapticDevice.CollisionMesh.transform.position, 0.1f);
            foreach (Collider collider in colliders)
            {
                if (collider.gameObject == box)
                {
                    objectType = "box";
                    newContact = true;
                    contactPoint = hapticDevice.LastContact;
                    break;
                }
                else if (collider.gameObject.transform.IsChildOf(robot1.transform))
                {
                    objectType = "robot1";
                    newContact = true;
                    contactPoint = hapticDevice.LastContact;
                    break;
                }
                else if (collider.gameObject.transform.IsChildOf(robot2.transform))
                {
                    objectType = "robot2";
                    newContact = true;
                    contactPoint = hapticDevice.LastContact;
                    break;
                }
            }
        }

        // Handle contact state changes
        if (newContact)
        {
            if (!isInContact || objectType != currentContactType)
            {
                // End previous contact if exists
                if (currentContact != null)
                {
                    EndCurrentContact();
                }
                
                // Start new contact
                StartNewContact(contactPoint, objectType);
            }

            // Update current contact duration
            if (currentContact != null)
            {
                currentContact.duration = currentTaskTime - currentContact.startTime;
            }
            
            // Sample force data
            if (Time.time - lastForceSampleTime >= forceSamplingRate)
            {
                UpdateContactForce();
                lastForceSampleTime = Time.time;
            }
        }
        else if (isInContact)
        {
            EndCurrentContact();
        }
    }

    void StartNewContact(Vector3 contactPoint, string objectType)
    {
        currentContact = new HapticContact();
        currentContact.Initialize(currentTaskTime, contactPoint, objectType);
        isInContact = true;
        currentContactType = objectType;
    }

    void UpdateContactForce()
    {
        if (currentContact != null)
        {
            float currentForce = hapticDevice.MagForce;
            currentContact.forceValues.Add(currentForce);
            currentContact.maxForce = Mathf.Max(currentContact.maxForce, currentForce);
        }
    }

    void EndCurrentContact()
    {
        if (currentContact != null)
        {
            currentContact.endTime = currentTaskTime;
            currentContact.duration = currentContact.endTime - currentContact.startTime;
            
            // Calculate average force
            if (currentContact.forceValues.Count > 0)
            {
                float sum = 0f;
                foreach (float force in currentContact.forceValues)
                {
                    sum += force;
                }
                currentContact.averageForce = sum / currentContact.forceValues.Count;
            }

            // Record contact data
            WriteContactData(currentContact);
            
            contactHistory.Add(currentContact);
            cumulativeContactTime += currentContact.duration;
        }

        isInContact = false;
        currentContactType = "";
        currentContact = null;
    }

    void WriteContactData(HapticContact contact)
    {
        contactWriter.WriteLine(string.Format(
            "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}",
            contactHistory.Count,
            contact.objectType,
            contact.startTime,
            contact.endTime,
            contact.duration,
            contact.maxForce,
            contact.averageForce,
            contact.contactPoint.x,
            contact.contactPoint.y,
            contact.contactPoint.z
        ));
    }

    void RecordDataPoint()
    {
        if (writer == null) return;

        // Box metrics
        float boxRotation = box.transform.eulerAngles.y;
        if (boxRotation > 180f) boxRotation -= 360f;
        float rotationError = Mathf.Abs(boxRotation);
        Vector3 boxPosition = box.transform.position;
        Vector3 boxAngularVelocity = box.GetComponent<Rigidbody>().angularVelocity;

        // Update performance metrics
        totalRotationError += rotationError;
        rotationErrorSamples++;
        maxRotationDeviation = Mathf.Max(maxRotationDeviation, rotationError);
        if (rotationError > rotationThreshold)
            stabilityViolations++;

        // Robot metrics
        Vector3 robot1Position = robot1.transform.position;
        Vector3 robot2Position = robot2.transform.position;
        float robot1Speed = robot1.GetComponent<ArticulationBody>().velocity.magnitude;
        float robot2Speed = robot2.GetComponent<ArticulationBody>().velocity.magnitude;
        float robotDistance = Vector3.Distance(robot1Position, robot2Position);

        // Haptic metrics
        Vector3 hapticPosition = hapticDevice.transform.position;
        Vector3 hapticForce = hapticDevice.CurrentForce;
        float forceMagnitude = hapticDevice.MagForce;

        // Update experiment phase
        string phase = UpdateExperimentPhase(boxRotation, rotationError);

        // Calculate stability metric
        float stabilityMetric = CalculateStabilityMetric(rotationError, forceMagnitude);

        // Get current contact duration
        float currentDuration = 0f;
        if (isInContact && currentContact != null)
        {
            currentDuration = currentTaskTime - currentContact.startTime;
        }

        // Write data point
        writer.WriteLine(string.Format(
            "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17}," +
            "{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30}",
            currentTaskTime,
            boxRotation, rotationError,
            boxPosition.x, boxPosition.y, boxPosition.z,
            boxAngularVelocity.x, boxAngularVelocity.y, boxAngularVelocity.z,
            robot1Position.x, robot1Position.y, robot1Position.z,
            robot2Position.x, robot2Position.y, robot2Position.z,
            robot1Speed, robot2Speed, robotDistance,
            hapticPosition.x, hapticPosition.y, hapticPosition.z,
            hapticForce.x, hapticForce.y, hapticForce.z,
            forceMagnitude,
            isInContact ? 1 : 0,
            currentContactType,
            currentDuration,
            phase,
            totalRotationError / (rotationErrorSamples > 0 ? rotationErrorSamples : 1),
            stabilityMetric
        ));
    }

    private string UpdateExperimentPhase(float boxRotation, float rotationError)
    {
        if (!isInContact)
            return "initialization";
        else if (isInContact && rotationError < rotationThreshold)
            return "stable_contact";
        else if (isInContact && rotationError >= rotationThreshold)
            return "correction";
        else
            return "release";
    }

    private float CalculateStabilityMetric(float rotationError, float forceMagnitude)
    {
        float normalizedError = rotationError / 180f;
        float normalizedForce = forceMagnitude / 5f;
        return (normalizedError + normalizedForce) / 2f;
    }

    public void StopRecording()
    {
        if (!isRecording) return;

        isRecording = false;
        float finalTaskTime = Time.time - experimentStartTime;

        if (currentContact != null)
        {
            EndCurrentContact();
        }

        if (writer != null)
        {
            WriteSessionSummary(finalTaskTime);
            writer.Close();
            writer = null;
        }

        if (contactWriter != null)
        {
            contactWriter.Close();
            contactWriter = null;
        }
        
        Debug.Log($"Stopped recording session: {sessionID}");
    }

    void WriteSessionSummary(float finalTaskTime)
    {
        writer.WriteLine("\nSession Summary");
        writer.WriteLine($"Total Task Time: {finalTaskTime:F2} seconds");
        writer.WriteLine($"Average Rotation Error: {(rotationErrorSamples > 0 ? totalRotationError / rotationErrorSamples : 0):F2} degrees");
        writer.WriteLine($"Maximum Rotation Deviation: {maxRotationDeviation:F2} degrees");
        writer.WriteLine($"Stability Violations: {stabilityViolations}");
        writer.WriteLine($"Total Contact Time: {cumulativeContactTime:F2} seconds");
        writer.WriteLine($"Contact Percentage: {(cumulativeContactTime/finalTaskTime*100):F2}%");
        
        // Contact statistics by type
        Dictionary<string, int> contactsByType = new Dictionary<string, int>();
        foreach (var contact in contactHistory)
        {
            if (!contactsByType.ContainsKey(contact.objectType))
                contactsByType[contact.objectType] = 0;
            contactsByType[contact.objectType]++;
        }
        
        writer.WriteLine("\nContact Statistics:");
        foreach (var kvp in contactsByType)
        {
            writer.WriteLine($"{kvp.Key}: {kvp.Value} contacts");
        }
    }

    void OnDisable()
    {
        if (isRecording)
        {
            StopRecording();
        }
    }

    // Public methods for accessing metrics
    public Dictionary<string, float> GetCurrentMetrics()
    {
        return new Dictionary<string, float>
        {
            {"TaskTime", currentTaskTime},
            {"AverageRotationError", rotationErrorSamples > 0 ? totalRotationError / rotationErrorSamples : 0},
            {"MaxRotationDeviation", maxRotationDeviation},
            {"StabilityViolations", stabilityViolations},
            {"TotalContacts", contactHistory.Count}
        };
    }

    public List<HapticContact> GetContactHistory()
    {
        return new List<HapticContact>(contactHistory);
    }
}