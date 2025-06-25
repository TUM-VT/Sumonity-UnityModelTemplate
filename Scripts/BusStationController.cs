using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace tum_car_controller
{
    public class BusStationController : MonoBehaviour
    {
        // Fields
        private BoxCollider busStationCollider;

        // Flags
        private bool busInStation = false;
        private bool arrived = false;
        private int frameCounter = 0;

        // Bus
        private GameObject bus;
        private float busSpeed = 0f;
        private const string busWheelTag = "BusWheel";
        // Why BusWheel? Because the colliders are on the wheels of the bus
        // If the collider was on the bus itself, it will have interfered with boarding

        // Unity Coroutines
        void Start()
        {
            busStationCollider = GetComponent<BoxCollider>();
            if (busStationCollider == null)
            {
                Debug.LogError("BusStationController requires a BoxCollider component.");
            }
            else
            {
                busStationCollider.isTrigger = true; // Ensure the collider is set as a trigger
            }
        }

        void FixedUpdate()
        {   
            if (bus != null)
            {
                if (busInStation)
                {
                    busSpeed = bus.GetComponent<BusController>().currentSpeed;
                    Debug.Log("Bus Speed: " + busSpeed);
                    DoorManager busDoorManager = bus.GetComponent<DoorManager>();

                    if (busSpeed == 0)
                        frameCounter++;
                    else
                        frameCounter = 0; // Reset if bus is moving
                    Debug.Log("Frame Counter: " + frameCounter);

                    if (busSpeed == 0 && arrived == false && frameCounter > 10)
                    {
                        arrived = true;
                        busDoorManager.openDoorsBasedOnScene();
                        Debug.Log("Bus has arrived at the station and doors are opening.");
                    }
                    else if (busSpeed > 0 && arrived == true) // leaving the station
                    {
                        arrived = false; // Reset for next arrival
                        busDoorManager.CloseDoorsBasedOnScene();
                        Debug.Log("Bus is moving out of the station area.");
                    }
                }

            }
        }

        /// Triggers
        private void OnTriggerEnter(Collider other)
        {

            if (other.CompareTag(busWheelTag))
            {
                busInStation = true;
                bus = other.transform.root.gameObject;
                Debug.Log("Bus has entered the station area.");
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.CompareTag(busWheelTag))
            {
                Debug.Log("Bus has exited the station area.");
                bus = null;
                busInStation = false;
            }
        }

        // Utility Methods
    }
}