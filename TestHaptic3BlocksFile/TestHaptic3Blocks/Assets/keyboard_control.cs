using UnityEngine;

public class keyboard_control : MonoBehaviour
{
    public float moveSpeed = 0.5f;
    public float turnSpeed = 100.0f;

    void Update()
    {
        // Movement control
        if (Input.GetKey(KeyCode.W))
        {
            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S))
        {
            transform.Translate(Vector3.back * moveSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.A))
        {
            transform.Rotate(Vector3.up, -turnSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D))
        {
            transform.Rotate(Vector3.up, turnSpeed * Time.deltaTime);
        }

        // Speed control
        if (Input.GetKeyDown(KeyCode.Q))
        {
            moveSpeed += 0.1f;
            Debug.Log("Speed increased to: " + moveSpeed);
        }
        if (Input.GetKeyDown(KeyCode.E))
        {
            moveSpeed -= 0.1f;
            if (moveSpeed < 0)
            {
                moveSpeed = 0;
            }
            Debug.Log("Speed decreased to: " + moveSpeed);
        }

        // Optional: motor control
        if (Input.GetKeyDown(KeyCode.Z))
        {
            StartMotors();
        }
        if (Input.GetKeyDown(KeyCode.X))
        {
            StopMotors();
        }
    }

    private void StartMotors()
    {
        Debug.Log("Starting motors");
        // Implement motor start logic here
    }

    private void StopMotors()
    {
        Debug.Log("Stopping motors");
        // Implement motor stop logic here
    }
}
