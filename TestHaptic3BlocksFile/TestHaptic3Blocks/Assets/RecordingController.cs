using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class RecordingController : MonoBehaviour
{
    [Header("References")]
    public ExperimentDataCollector dataCollector;
    public TextMeshProUGUI buttonText;
    public Button recordButton;
    
    [Header("Button Text")]
    public string startRecordText = "Start Recording";
    public string stopRecordText = "Stop Recording";
    
    [Header("Button Colors")]
    public Color startColor = new Color(0.2f, 0.8f, 0.2f); // Green
    public Color stopColor = new Color(0.8f, 0.2f, 0.2f);  // Red
    
    private void Start()
    {
        // Ensure references are set
        if (dataCollector == null)
        {
            dataCollector = FindObjectOfType<ExperimentDataCollector>();
            if (dataCollector == null)
            {
                Debug.LogError("ExperimentDataCollector not found!");
                return;
            }
        }
        
        // Set up button
        if (recordButton == null)
        {
            recordButton = GetComponent<Button>();
        }
        
        // Set up initial button state
        buttonText.text = startRecordText;
        recordButton.onClick.AddListener(ToggleRecording);
        
        // Set initial colors
        UpdateButtonColors(startColor);
    }
    
    private void UpdateButtonColors(Color baseColor)
    {
        var colors = recordButton.colors;
        
        // Set normal color
        colors.normalColor = baseColor;
        
        // Create slightly darker variant for highlighted state
        Color.RGBToHSV(baseColor, out float h, out float s, out float v);
        colors.highlightedColor = Color.HSVToRGB(h, s, v * 0.9f);
        
        // Create even darker variant for pressed state
        colors.pressedColor = Color.HSVToRGB(h, s, v * 0.8f);
        
        // Create slightly transparent variant for disabled state
        colors.disabledColor = new Color(baseColor.r, baseColor.g, baseColor.b, 0.5f);
        
        // Adjust selected color
        colors.selectedColor = baseColor;
        
        // Update color transition settings
        colors.colorMultiplier = 1f;
        colors.fadeDuration = 0.1f;
        
        // Apply all color changes
        recordButton.colors = colors;
    }
    
    public void ToggleRecording()
    {
        if (!dataCollector.isActiveAndEnabled)
        {
            Debug.LogWarning("ExperimentDataCollector is not active!");
            return;
        }
        
        if (buttonText.text == startRecordText)
        {
            // Start recording
            dataCollector.StartRecording();
            buttonText.text = stopRecordText;
            UpdateButtonColors(stopColor);
        }
        else
        {
            // Stop recording
            dataCollector.StopRecording();
            buttonText.text = startRecordText;
            UpdateButtonColors(startColor);
        }
    }
    
    private void OnDisable()
    {
        if (recordButton != null)
        {
            recordButton.onClick.RemoveListener(ToggleRecording);
        }
    }
}