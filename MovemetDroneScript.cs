//FOR MANUAL DRIVING

/*
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;

public class DroneMovementScript : MonoBehaviour
{
    Rigidbody ourDrone;
    private List<Vector3> pathData = new List<Vector3>();
    private int framesPerPoint = 10; // Update path every X frames
    private int frameCounter = 0;

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
    private bool isColliding = false;

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
            MovementUpDown();
            MovementForward();
            Rotation();
            ClampingSpeedValues();
            Swerwe();

            frameCounter++;
            if (frameCounter >= framesPerPoint)
            {
                pathData.Add(transform.position);
                frameCounter = 0;
            }

            ourDrone.AddRelativeForce(Vector3.up * upForce);
            ourDrone.rotation = Quaternion.Euler(new Vector3(tiltAmountForward, currentYRotation, tiltAmountSideways));

            ExportPathData();
        }
    }

    public void ExportPathData()
    {
        // Check if the pathData list is not empty before exporting
        if (pathData.Count > 0)
        {
            Debug.Log("Exporting path data");

            string path = "Assets/path_data.csv"; // Adjust the path as needed

            // Write the data to a CSV file without parentheses
            System.IO.File.WriteAllLines(path, pathData.Select(p => $"{p.x},{p.y},{p.z}").ToArray());

            // Debug: Print CSV contents to Unity console
            string[] lines = System.IO.File.ReadAllLines(path);
            foreach (string line in lines)
            {
                Debug.Log(line);
            }
        }
    }




    public float upForce;
	void MovementUpDown(){
		
		if((Mathf.Abs(Input.GetAxis("Vertical")) > 0.2f || Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f)){
			if (Input.GetKey(KeyCode.I) || Input.GetKey(KeyCode.K)){
				ourDrone.velocity = ourDrone.velocity;
			}
			if(!Input.GetKey(KeyCode.I) && !Input.GetKey(KeyCode.K) && !Input.GetKey(KeyCode.J) && !Input.GetKey(KeyCode.L)){
				ourDrone.velocity = new Vector3(ourDrone.velocity.x, Mathf.Lerp(ourDrone.velocity.y, 0, Time.deltaTime * 5), ourDrone.velocity.z);
				upForce = 281;
			}
			if(!Input.GetKey(KeyCode.I) && !Input.GetKey(KeyCode.K) && (Input.GetKey(KeyCode.J) || Input.GetKey(KeyCode.L))){
				ourDrone.velocity = new Vector3(ourDrone.velocity.x, Mathf.Lerp(ourDrone.velocity.y, 0, Time.deltaTime * 5), ourDrone.velocity.z);
				upForce = 110;
			}
			if(Input.GetKey(KeyCode.J) || Input.GetKey(KeyCode.L)){
				upForce = 410;
			}
        }
		
		if((Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f)){
		    upForce = 135;
		}

		
		if(Input.GetKey(KeyCode.I)){
			upForce = 450;
			if(Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
				upForce = 500;
			}
		}
		else if(Input.GetKey(KeyCode.K)){
			upForce = -200;
		}
		else if(!Input.GetKey(KeyCode.I) && !Input.GetKey(KeyCode.K) && (Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) < 0.2f)){
			upForce = 98.1f;
		}
	}

	private float movementForwardSpeed = 500.0f;
    private float tiltAmountForward = 0;
    private float tiltVelocityForward;
	void MovementForward(){
	if (Input.GetAxis("Vertical") != 0){
	   ourDrone.AddRelativeForce(Vector3.forward * Input.GetAxis("Vertical") * movementForwardSpeed);
	   tiltAmountForward = Mathf.SmoothDamp(tiltAmountForward , 5 * Input.GetAxis("Vertical"), ref tiltVelocityForward, 0.1f);
	  }
	}


	private float wantedYRotation;
	[HideInInspector]public float currentYRotation;
	private float rotateAmountByKeys = 2.5f;
	private float rotationYVelocity;

	void Rotation(){
		if(Input.GetKey(KeyCode.J)){
		     wantedYRotation -= rotateAmountByKeys;
		}
		if(Input.GetKey(KeyCode.L)){
		     wantedYRotation += rotateAmountByKeys;
		}
		currentYRotation = Mathf.SmoothDamp(currentYRotation , wantedYRotation , ref rotationYVelocity, 0.25f);
	}

	private Vector3 velocityToSmoothDampToZero;
	void ClampingSpeedValues(){
	   if(Mathf.Abs(Input.GetAxis("Vertical")) > 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
	   	   ourDrone.velocity = Vector3.ClampMagnitude(ourDrone.velocity , Mathf.Lerp(ourDrone.velocity.magnitude, 10.0f, Time.deltaTime * 5f));
	   }
	   
	   if(Mathf.Abs(Input.GetAxis("Vertical")) > 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) < 0.2f){
	   	   ourDrone.velocity = Vector3.ClampMagnitude(ourDrone.velocity , Mathf.Lerp(ourDrone.velocity.magnitude, 10.0f, Time.deltaTime * 5f));
	   }

	   if(Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
	   	   ourDrone.velocity = Vector3.ClampMagnitude(ourDrone.velocity , Mathf.Lerp(ourDrone.velocity.magnitude, 5.0f, Time.deltaTime * 5f));
	   }

	   if(Mathf.Abs(Input.GetAxis("Vertical")) < 0.2f && Mathf.Abs(Input.GetAxis("Horizontal")) < 0.2f){
	   	   ourDrone.velocity = Vector3.SmoothDamp(ourDrone.velocity , Vector3.zero, ref velocityToSmoothDampToZero, 0.95f);
	   }
	}


	private float sideMovementAmount = 350.0f;
	private float tiltAmountSideways;
	private float tiltAmountVelocity;

	void Swerwe(){
		if(Mathf.Abs(Input.GetAxis("Horizontal")) > 0.2f){
			ourDrone.AddRelativeForce(Vector3.right * Input.GetAxis("Horizontal") * sideMovementAmount);
			tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, -20 * Input.GetAxis("Horizontal"), ref tiltAmountVelocity, 0.1f);
		}
		else{
		    tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, 0, ref tiltAmountVelocity, 0.1f);
		}
	}
    
	

  
}
*/
//FOR AUTONOMOUS DRIVING

/*
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;

public class DroneMovementScript : MonoBehaviour
{
    private bool autonomousNavigationEnabled = false;
    public float movementSpeed = 2f;
    public float rotationSpeed = 20f;
    private List<Vector3> dronePath = new List<Vector3>(); // List to store drone's path

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !autonomousNavigationEnabled)
        {
            autonomousNavigationEnabled = true;
            StartCoroutine(AutonomousNavigation());
        }
    }

    IEnumerator AutonomousNavigation()
    {
        Vector3 destination = new Vector3(-180f, 600f, 209f); // Set the desired destination coordinates

        Debug.Log("Starting autonomous movement to: " + destination);

        float startTime = Time.time;
        float journeyLength = Vector3.Distance(transform.position, destination);

        while (autonomousNavigationEnabled && (Time.time - startTime) < 10f) // Move for 10 seconds
        {
            // Perform a raycast in the drone's forward direction to detect obstacles
            if (Physics.Raycast(transform.position, transform.forward, out RaycastHit hit, 10f))
            {
                Debug.Log("Raycast hit: " + hit.collider.gameObject.name); // Log the hit object's name
                if (hit.collider.CompareTag("Obstacle"))
                {
                    // Implement obstacle avoidance logic here
                    AvoidObstacle();
                }
            }

            float distCovered = (Time.time - startTime) * movementSpeed;
            float fracJourney = distCovered / journeyLength;
            transform.position = Vector3.Lerp(transform.position, destination, fracJourney);

            // Record the current position in the drone's path
            dronePath.Add(transform.position);

            Debug.Log("Distance Covered: " + distCovered);
            Debug.Log("Fraction of Journey: " + fracJourney);

            yield return null;
        }

        Debug.Log("Autonomous movement completed.");
        autonomousNavigationEnabled = false; // Reset the flag

        Debug.Log("Current Position: " + transform.position);
        Debug.Log("Destination: " + destination);

        // Save the drone's path to a CSV file
        SaveDronePathToCSV();

        // Visualize LiDAR data (assuming you have a LiDAR simulation script attached)
        SimulateLiDAR();
    }

    void SaveDronePathToCSV()
    {
        string dronePathFilePath = "Assets/drone_path.csv";
        using (StreamWriter writer = new StreamWriter(dronePathFilePath))
        {
            foreach (Vector3 position in dronePath)
            {
                writer.WriteLine($"{position.x},{position.y},{position.z}");
            }
        }
    }

    void SimulateLiDAR()
    {
        // Simulate LiDAR rays in a circular pattern
        for (float angle = 0f; angle < 360f; angle += 10f) // Adjust the angle increment as needed
        {
            float radians = angle * Mathf.Deg2Rad;
            Vector3 rayDirection = new Vector3(Mathf.Sin(radians), 0f, Mathf.Cos(radians));
            rayDirection = transform.rotation * rayDirection;

            // Cast a ray from the LiDAR sensor
            Debug.DrawRay(transform.position, rayDirection * 10f, Color.yellow);
        }
    }

    bool ObstacleDetected()
    {
        // Check if any obstacles are detected based on your LiDAR simulation
        // For simplicity, this example assumes an obstacle is detected if any ray hits something
        for (float angle = 0f; angle < 360f; angle += 10f)
        {
            float radians = angle * Mathf.Deg2Rad;
            Vector3 rayDirection = new Vector3(Mathf.Sin(radians), 0f, Mathf.Cos(radians));
            rayDirection = transform.rotation * rayDirection;

            if (Physics.Raycast(transform.position, rayDirection, 10f))
            {
                // Obstacle detected
                return true;
            }
        }

        // No obstacles detected
        return false;
    }

    void AvoidObstacle()
    {
        // Rotate the drone to avoid the obstacle
        transform.Rotate(Vector3.up, rotationSpeed * Time.deltaTime);
    }
}
*/







/*
//FOR OBSTACLE AVOIDANCE
using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using System.Collections.Generic;
using System.IO;

public class DroneMovementScript : MonoBehaviour
{
    private bool autonomousNavigationEnabled = false;
    private NavMeshAgent navMeshAgent; // Reference to the NavMeshAgent
    private Vector3 originalDestination;
    private float obstacleAvoidanceRadius = 20f; // Radius to check for obstacles
    private float backupDistance = 5f; // Distance to back up upon detecting an obstacle
    private bool isBackingUp = false;
    private GameObject lidarSensor;
    private List<GameObject> lidarPoints = new List<GameObject>();

    void Start()
    {
        // Get the NavMeshAgent component attached to the drone
        navMeshAgent = GetComponent<NavMeshAgent>();
        // Enable high-quality obstacle avoidance
        navMeshAgent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
        InitializeLiDARSensor();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !autonomousNavigationEnabled)
        {
            autonomousNavigationEnabled = true;
            StartCoroutine(AutonomousNavigation());
        }
        foreach (GameObject lidarPoint in lidarPoints)
        {
            float distance = Vector3.Distance(transform.position, lidarPoint.transform.position);
            Debug.Log($"Distance from Drone to Obstacle: {distance} units");
        }
    }
    void InitializeLiDARSensor()
    {
        lidarSensor = new GameObject("LiDAR Sensor");

        // Read LiDAR data from the CSV file
        string lidarDataFilePath = "Assets/lidar_simulation.csv"; // Provide the correct path
        string[] lines = File.ReadAllLines(lidarDataFilePath);
        Debug.Log("Attempting to load file from: " + lidarDataFilePath);

        foreach (string line in lines)
        {
            // Parse the line and extract LiDAR point information
            string[] values = line.Split(',');

            if (values.Length >= 3 && float.TryParse(values[0], out float x) && float.TryParse(values[1], out float y) && float.TryParse(values[2], out float z))
            {
                // Log LiDAR point information for debugging
                Debug.Log($"LiDAR Point: X={x}, Y={y}, Z={z}");

                // Instantiate Spheres for LiDAR points
                GameObject lidarPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                lidarPoint.transform.position = new Vector3(x, y, z);

                // Parent the LiDAR points to the LiDAR Sensor
                lidarPoint.transform.parent = lidarSensor.transform;

                // Add the lidarPoint to the list for future reference
                lidarPoints.Add(lidarPoint);
            }
            else
            {
                Debug.LogError("Error parsing LiDAR data: " + line);
            }
        }
    }

    IEnumerator AutonomousNavigation()
    {
        Vector3 destination = new Vector3(25f, 1f, -0.5f);
        Debug.Log("Starting autonomous movement to: " + destination);

        // Set the original destination for the NavMeshAgent
        originalDestination = destination;
        navMeshAgent.SetDestination(destination);

        while (autonomousNavigationEnabled)
        {
            // Check if there's an obstacle along the current path
            if (IsPathBlocked())
            {
                // Back up and find an alternative path
                yield return StartCoroutine(BackupAndFindAlternativePath());
            }

            // Check if the drone has reached the original destination
            if (!isBackingUp && Vector3.Distance(transform.position, originalDestination) <= navMeshAgent.stoppingDistance)
            {
                Debug.Log("Autonomous movement completed.");
                autonomousNavigationEnabled = false;
                break;
            }
            yield return null; // Yield control back to Unity
        }

        // Save the drone's path to a CSV file
        SaveDronePathToCSV();

        // Visualize LiDAR data (assuming you have a LiDAR simulation script attached)
        SimulateLiDAR();
    }

    bool IsPathBlocked()
    {
        // Cast a ray forward to check for obstacles
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, obstacleAvoidanceRadius))
        {
            // Check if the hit object is an obstacle
            if (hit.collider.gameObject.CompareTag("Obstacle"))
            {
                return true;
            }
        }

        return false;
    }

    IEnumerator BackupAndFindAlternativePath()
    {
        if (!isBackingUp)
        {
            // Back up by a certain distance
            Vector3 backupPosition = transform.position - transform.forward * backupDistance;
            navMeshAgent.SetDestination(backupPosition);
            isBackingUp = true;
        }

        // Rotate the drone to find an alternative path
        float randomRotation = Random.Range(90f, 270f); // Rotate randomly between 90 and 270 degrees
        transform.Rotate(Vector3.up, randomRotation);

        // Set a new destination based on the rotated direction
        Vector3 newDestination = transform.position + transform.forward * obstacleAvoidanceRadius;
        navMeshAgent.SetDestination(newDestination);

        // Check if the drone has finished backing up
        while (isBackingUp && navMeshAgent.remainingDistance > navMeshAgent.stoppingDistance)
        {
            yield return null; // Yield control back to Unity
        }

        isBackingUp = false;
    }

    void SaveDronePathToCSV()
    {
        string dronePathFilePath = "Assets/drone_path.csv";
        using (StreamWriter writer = new StreamWriter(dronePathFilePath))
        {
            // Save the drone's path based on its actual path
            foreach (Vector3 position in navMeshAgent.path.corners)
            {
                writer.WriteLine($"{position.x},{position.y},{position.z}");
            }
        }
    }

    void SimulateLiDAR()
    {
        // Simulate LiDAR rays in a circular pattern
        for (float angle = 0f; angle < 360f; angle += 10f)
        {
            float radians = angle * Mathf.Deg2Rad;
            Vector3 rayDirection = new Vector3(Mathf.Sin(radians), 0f, Mathf.Cos(radians));
            rayDirection = transform.rotation * rayDirection;

            // Cast a ray from the LiDAR sensor
            Debug.DrawRay(transform.position, rayDirection * 10f, Color.yellow);
        }
    }
}

*/



using UnityEngine;
using System.Linq; // For sorting waypoints

public class DroneNavigation : MonoBehaviour
{
    public float forwardSpeed = 2f; // Speed of forward movement
    public float rayLength = 20f; // Length of the LIDAR ray
    public float minDistanceToDestination = 1f; // Distance to consider the destination reached
    public float ascendHeightIncrement = 5f; // Ascend by 5 meters incrementally
    public float raySpacing = 1f; // Spacing between rays

    private GameObject[] waypoints; // Array to store all waypoints
    private int currentWaypointIndex = 0; // Current waypoint the drone is moving towards
    private bool hasReachedDestination = false;

    void Start()
    {
        // Get all waypoints tagged as "Destination"
        waypoints = GameObject.FindGameObjectsWithTag("Destination");

        if (waypoints.Length == 0)
        {
            Debug.LogError("No waypoints found with the tag 'Destination'.");
            hasReachedDestination = true;
            return;
        }

        // Sort the waypoints based on their name (assumes naming convention "map pointer", "map pointer (1)", "map pointer (2)", etc.)
        waypoints = waypoints.OrderBy(w => {
            string name = w.name;
            // Handle the base case "map pointer" (no index in the name)
            if (name == "map pointer") return 0;

            // Extract the number in parentheses, e.g., "(1)" from "map pointer (1)"
            int startIndex = name.IndexOf('(');
            if (startIndex != -1)
            {
                string number = name.Substring(startIndex + 1, name.Length - startIndex - 2);
                if (int.TryParse(number, out int result))
                {
                    return result;
                }
            }

            // If parsing fails or no number is found, assign a large value to push it to the end
            return int.MaxValue;
        }).ToArray();
    }

    void Update()
    {
        if (hasReachedDestination)
        {
            Debug.Log("Drone has reached all waypoints and stopped.");
            return;
        }

        // Check if the drone has reached the current waypoint
        if (Vector3.Distance(transform.position, waypoints[currentWaypointIndex].transform.position) < minDistanceToDestination)
        {
            Debug.Log($"Waypoint {currentWaypointIndex + 1} reached!");
            currentWaypointIndex++;

            // If all waypoints have been visited, stop the drone
            if (currentWaypointIndex >= waypoints.Length)
            {
                hasReachedDestination = true;
                Debug.Log("All waypoints visited!");
                return;
            }
        }

        // Draw rays around the drone for visualization
        DrawSurroundingRays();

        // Continuous LIDAR-based obstacle detection
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, rayLength))
        {
            if (IsObstacle(hit.collider))
            {
                Debug.DrawRay(transform.position, transform.forward * rayLength, Color.red); // Obstacle detected, ray turns red
                Debug.Log($"Obstacle detected! Object: {hit.collider.name}");

                // Ascend by increment until there are no obstacles
                AscendByIncrement();
            }
            else
            {
                Debug.DrawRay(transform.position, transform.forward * rayLength, Color.green); // No obstacle, ray turns green
                Debug.Log("No obstacle detected.");
                MoveTowardsCurrentWaypoint();
            }
        }
        else
        {
            // If no hit is detected, assume clear path and move toward current waypoint
            Debug.DrawRay(transform.position, transform.forward * rayLength, Color.green);
            MoveTowardsCurrentWaypoint();
        }
    }

    void MoveTowardsCurrentWaypoint()
    {
        // Get the current waypoint's position
        Vector3 destination = waypoints[currentWaypointIndex].transform.position;

        // Calculate direction
        Vector3 direction = (destination - transform.position).normalized;

        // Set the drone's rotation: Lock X and Z, rotate freely on Y-axis
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Euler(0, targetRotation.eulerAngles.y, 0); // Only lock X and Z rotations

        // Move forward
        transform.position += transform.forward * forwardSpeed * Time.deltaTime;
        Debug.Log($"Moving towards waypoint {currentWaypointIndex + 1}.");
    }


    void AscendByIncrement()
    {
        Debug.Log("Ascending by 5 meters...");
        // Ascend by 5 meters
        transform.position += new Vector3(0, ascendHeightIncrement, 0);

        // Check for obstacles again immediately after ascending
        CheckForObstaclesAfterAscend();
    }

    void CheckForObstaclesAfterAscend()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, rayLength))
        {
            if (IsObstacle(hit.collider))
            {
                Debug.Log("Obstacle still detected after ascending. Continuing to ascend...");
                AscendByIncrement();
            }
            else
            {
                Debug.Log("Obstacle cleared after ascending. Resuming movement towards waypoint.");
                MoveTowardsCurrentWaypoint();
            }
        }
        else
        {
            Debug.Log("No obstacle detected after ascending. Resuming movement towards waypoint.");
            MoveTowardsCurrentWaypoint();
        }
    }

    private bool IsObstacle(Collider collider)
    {
        if (collider.CompareTag("Obstacle"))
        {
            return true;
        }

        Transform current = collider.transform.parent;
        while (current != null)
        {
            if (current.CompareTag("Obstacle"))
            {
                return true;
            }
            current = current.parent;
        }
        return false;
    }

    private void DrawSurroundingRays()
    {
        for (int i = -2; i <= 3; i++) // Total 6 rays: -2, -1, 0, 1, 2, 3
        {
            Vector3 rayOrigin = transform.position + transform.right * raySpacing * i; // Offset by raySpacing
            Debug.DrawRay(rayOrigin, transform.forward * rayLength, Color.yellow);
        }
    }
}
