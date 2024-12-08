using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class QuadrotorTogglePublisher : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>("/unity_movement_feedback");
        ros.RegisterPublisher<TwistMsg>("/quadrotor/cmd_vel");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.W) || Input.GetKeyDown(KeyCode.S)) {
            ToggleMovement();
        }

    }

    void ToggleMovement() {
        Debug.Log("Unity: Toggling Movement");
        
        TwistMsg msg = new TwistMsg();
        msg.linear.y = 0.0f;

        if (Input.GetKeyDown(KeyCode.W)) {
            msg.linear.y = 0.2f;
        }
        else if (Input.GetKeyDown(KeyCode.S)) {
            msg.linear.y = -0.2f;

        }

        ros.Publish("/quadrotor/cmd_vel", msg);

        Debug.Log($"Published linear.y = {msg.linear.y}");
    }     
}
