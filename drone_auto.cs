/*using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class DroneMovementScript : MonoBehaviour
{
    Rigidbody ourDrone;
    private bool isColliding = false;
    private bool autonomousNavigationEnabled = false; // Set this to true to enable autonomous navigation

    // Other variables and functions...

    void Awake()
    {
        ourDrone = GetComponent<Rigidbody>();
        if (ourDrone == null)
        {
            ourDrone = gameObject.AddComponent<Rigidbody>();
        }
        ourDrone.collisionDetectionMode = CollisionDetectionMode.Continuous;
    }

    private float collisionCooldown = 1f;
    private float lastCollisionTime;

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            isColliding = true;
            ourDrone.velocity = Vector3.zero;
        }
    }

    void FixedUpdate()
    {
        if (!isColliding)
        {
            // Check if autonomous navigation is enabled
            if (autonomousNavigationEnabled)
            {
                // Call the function to perform autonomous navigation
                AutonomousNavigation();
            }
            else
            {
                // If autonomous navigation is not enabled, perform manual control
                MovementUpDown();
                MovementForward();
                Rotation();
                ClampingSpeedValues();
                Swerwe();

                ourDrone.AddRelativeForce(Vector3.up * upForce);
                ourDrone.rotation = Quaternion.Euler(new Vector3(tiltAmountForward, currentYRotation, tiltAmountSideways));
            }
        }
    }

    // Add this function for autonomous navigation
    void AutonomousNavigation()
    {
        // Get the destination point (for example, the point where the user clicked)
        Vector3 destinationPoint = GetDestinationPoint();

        // Implement pathfinding to calculate a collision-free path
        List<Vector3> path = CalculatePath(transform.position, destinationPoint);

        // Navigate the drone along the calculated path, avoiding obstacles
        Navigate(path);
    }

    // Example function to get the destination point (replace this with your logic)
    Vector3 GetDestinationPoint()
    {
        // For example, you could replace this with a function that returns the point where the user clicked
        return new Vector3(10f, 0f, 10f);
    }

    // Example pathfinding function (replace this with your pathfinding logic)
    List<Vector3> CalculatePath(Vector3 start, Vector3 destination)
    {
        // Replace this with your pathfinding logic (e.g., A* algorithm)
        List<Vector3> path = new List<Vector3>();
        path.Add(start);
        path.Add(destination);
        return path;
    }

    // Example obstacle avoidance function (replace this with your obstacle avoidance logic)
    bool ObstacleDetected(Vector3 position)
    {
        // Replace this with your obstacle detection logic
        // For example, cast rays or use collision detection to check for obstacles
        return false;
    }

    // Other functions...

    // Rest of your existing code for manual control...
}
*/