using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.AI.Navigation;
using UnityEngine.AI;


namespace tum_car_controller
{
    public class BusStationController : MonoBehaviour
    {
        // Fields
        private BoxCollider busStationCollider;
        private NavMeshSurface busStationNavMeshSurface;
        private NavMeshSurface busFloorNavMeshSurface;
        private NavMeshSurface busFrontRampNavMeshSurface;
        private NavMeshSurface busRearRampNavMeshSurface;


        // Flags
        private bool busInStation = false;
        private bool startedBoarding = false;
        private bool endedBoarding = false;
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
                    DoorManager busDoorManager = bus.GetComponent<DoorManager>();

                    if (!startedBoarding && !endedBoarding)
                    {
                        // We are checking if the bus is stationary only when it is in the station and has not started boarding yet
                        if (busSpeed == 0)
                            frameCounter++;
                        else
                            frameCounter = 0; // Reset if bus is moving

                        if (busSpeed == 0 && frameCounter > 10)
                        {
                            // Debug.Log("Bus has arrived at the station and doors are opening.");
                            startedBoarding = true;
                            busDoorManager.openDoorsBasedOnScene();
                            Invoke(nameof(BakeNavMesh), 0.5f);
                        }
                    }
                    else if (busSpeed > 0 && startedBoarding && !endedBoarding) // leaving the station
                    {
                        // Debug.Log("StartedBoarding: " + startedBoarding + ", EndedBoarding: " + endedBoarding);
                        // Debug.Log("Bus is moving out of the station area. Closing doors.");
                        endedBoarding = true;
                        busDoorManager.CloseDoorsBasedOnScene();
                    }
                }
                else if (startedBoarding && endedBoarding)
                {
                    startedBoarding = false;
                    endedBoarding = false;
                    frameCounter = 0; // Reset frame counter
                    // Debug.Log("Resetting boarding state as bus has left the station.");
                }
            }
        }

        /// Triggers
        private void OnTriggerEnter(Collider other)
        {

            if (other.CompareTag(busWheelTag))
            {
                Debug.Log("Bus has entered the station area.");
                busInStation = true;
                bus = other.transform.root.gameObject;
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

        void BakeNavMesh()
        {
            // 1. Find the transforms of all relevant objects
            Transform busStation = this.transform;
            Transform busFloor = bus.transform.Find("BusFloor");
            Transform frontRamp = bus.transform.Find("FrontRamp");
            Transform rearRamp = bus.transform.Find("RearRamp");

            // 2. Store original parents
            Transform[] objects = { busStation, busFloor, frontRamp, rearRamp };
            Transform[] originalParents = new Transform[objects.Length];
            for (int i = 0; i < objects.Length; i++)
                originalParents[i] = objects[i].parent;

            // 3. Create temporary parent
            GameObject tempParent = new GameObject("TempNavMeshParent");
            foreach (var obj in objects)
                obj.parent = tempParent.transform;

            // 4. Add NavMeshSurface to temp parent and bake
            var navMeshSurface = tempParent.AddComponent<NavMeshSurface>();
            navMeshSurface.collectObjects = CollectObjects.Children;
            navMeshSurface.BuildNavMesh();

            // 5. Restore original parents
            for (int i = 0; i < objects.Length; i++)
                objects[i].parent = originalParents[i];

            // 6. Destroy the temporary parent (NavMesh remains)
            // Destroy(tempParent)
        }

    }
}