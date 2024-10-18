/*using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollowScript : MonoBehaviour {
    private Transform ourDrone;

   void Awake() {
    ourDrone = GameObject.FindGameObjectWithTag("Player").transform;
    if (ourDrone == null) {
        Debug.LogError("Player object not found with tag 'Player'.");
    } else {
        Debug.Log("Player object found with tag 'Player'.");
    }
}


    private Vector3 velocityCameraFollow;
    public Vector3 behindPosition = new Vector3(0, 2, -4);
    public float angle;

    void FixedUpdate() {
        if (ourDrone != null) {
            Transform droneTransform = ourDrone.transform;

            DroneMovementScript droneScript = ourDrone.GetComponent<DroneMovementScript>();
            if (droneScript == null) {
                Debug.LogError("DroneMovementScript component not found on the drone.");
                return;
            }

            transform.position = Vector3.SmoothDamp(transform.position, droneTransform.TransformPoint(behindPosition) + Vector3.up * Input.GetAxis("Vertical"), ref velocityCameraFollow, 0.1f);
            transform.rotation = Quaternion.Euler(new Vector3(angle, droneScript.currentYRotation, 0));
        } else {
            Debug.LogError("ourDrone is null in FixedUpdate.");
        }
    }
}*/
