using System.Collections;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

public class DroneFlight : MonoBehaviour
{
    public Transform[] waypoints;  // Array of waypoints
    public float speed = 5f;  // Drone movement speed
    public float heightSpeed = 2f;  // Height adjustment speed
    public float reachThreshold = 0.1f;  // Waypoint reach threshold
    public float avoidanceDistance = 3f;  // Distance to move for obstacle avoidance

    private int currentWaypointIndex = 0;  // Current waypoint index
    private bool hasCompletedRound = false;  // Flag for round completion
    private bool hasRotated = false;  // Rotation flag
    private bool avoidingObstacle = false;  // Flag to check if avoiding obstacle

    // Data to store flight and obstacle info
    private List<string> flightData = new List<string>();  // Stores drone positions and obstacle info

    void Start()
    {
        // Initialize flight data list with a header
        flightData.Add("X,Y,Z,ObstacleName,ObstaclePosition,ObstacleSize,ObstacleMaterial");
    }

    void Update()
    {
        if (waypoints.Length == 0 || hasCompletedRound)
            return;

        // Check for obstacles and handle avoidance
        if (ObstacleInPath() && !avoidingObstacle)
        {
            AvoidObstacle();  // Handle obstacle avoidance
            return;  // Stop further movement towards the waypoint during avoidance
        }

        // Normal waypoint following logic
        Transform targetWaypoint = waypoints[currentWaypointIndex];

        // Step 1: Adjust height
        Vector3 targetPosition = new Vector3(transform.position.x, targetWaypoint.position.y, transform.position.z);
        transform.position = Vector3.MoveTowards(transform.position, targetPosition, heightSpeed * Time.deltaTime);

        // Step 2: Move towards the waypoint
        if (Mathf.Abs(transform.position.y - targetWaypoint.position.y) < reachThreshold)
        {
            transform.position = Vector3.MoveTowards(transform.position, targetWaypoint.position, speed * Time.deltaTime);

            // Check if drone reached the waypoint
            if (Vector3.Distance(transform.position, targetWaypoint.position) < reachThreshold)
            {
                // Rotate 90 degrees after waypoint, except for the first one
                if (currentWaypointIndex > 0 && !hasRotated)
                {
                    transform.Rotate(0, 0, 90);
                    hasRotated = true;
                }

                // Move to the next waypoint
                currentWaypointIndex++;

                if (currentWaypointIndex == waypoints.Length)
                {
                    hasCompletedRound = true;
                    ExportFlightData();  // Export data when round is complete
                    return;
                }

                hasRotated = false;
            }
        }

        // Record flight data
        RecordFlightData();
    }

    // Raycasting for obstacle detection
    bool ObstacleInPath()
    {
        RaycastHit hit;
        float rayDistance = 5f;

        if (Physics.Raycast(transform.position, transform.forward, out hit, rayDistance))
        {
            // Get obstacle details
            string obstacleName = hit.collider.name;
            Vector3 obstaclePosition = hit.collider.transform.position;
            Vector3 obstacleSize = hit.collider.bounds.size;

            // Optionally, get the material (if any) from the object's renderer
            Renderer renderer = hit.collider.GetComponent<Renderer>();
            string obstacleMaterial = renderer != null ? renderer.material.name : "Unknown";

            // Record obstacle data along with current drone position
            flightData.Add($"{transform.position.x},{transform.position.y},{transform.position.z},{obstacleName},{obstaclePosition},{obstacleSize},{obstacleMaterial}");
            return true;
        }

        return false;
    }

    // Simple avoidance logic: move sideways or up
    void AvoidObstacle()
    {
        avoidingObstacle = true;

        // Choose an avoidance direction (here we move sideways to the right)
        Vector3 avoidanceDirection = Vector3.right * avoidanceDistance;

        // Check if there is enough space to move sideways, otherwise move up
        if (!Physics.Raycast(transform.position, avoidanceDirection, avoidanceDistance))
        {
            transform.position += avoidanceDirection;  // Move to the side
        }
        else
        {
            transform.position += Vector3.up * avoidanceDistance;  // Move up if sideways blocked
        }

        avoidingObstacle = false;  // Reset after avoidance
    }

    // Record flight data (when no obstacle detected)
    void RecordFlightData()
    {
        flightData.Add($"{transform.position.x},{transform.position.y},{transform.position.z},None,None,None,None");
    }

    // Export flight and obstacle data to CSV
    void ExportFlightData()
    {
        string path = Application.dataPath + "/flightData.csv";

        using (StreamWriter writer = new StreamWriter(path))
        {
            foreach (string data in flightData)
            {
                writer.WriteLine(data);
            }
        }

        Debug.Log("Flight and obstacle data exported to: " + path);
    }
}
