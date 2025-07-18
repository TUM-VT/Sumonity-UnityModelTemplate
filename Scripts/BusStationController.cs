using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.AI.Navigation;
using UnityEngine.AI;
using tumvt.sumounity.PedestrianModel;

namespace tum_bus_controller
{
    public class BusStationController : MonoBehaviour
    {
        // Fields
        private BoxCollider busStationCollider;

        // For dynamic NavMesh
        public GameObject tempNavMeshSurfaceObj = null;
        public NavMeshSurface tempNavMeshSurface = null;
        // private Transform[] navMeshObjects = null;
        // private Transform[] originalParents = null;

        // Flags
        private bool busInStation = false;
        private bool startedBoarding = false;
        private bool endedBoarding = false;
        private int frameCounter = 0;

        // Bus
        private GameObject bus;
        private float busSpeed = 0f;
        private const string busWheelTag = "BusWheel";

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
                            startedBoarding = true;
                            busDoorManager.openDoorsBasedOnScene();
                            Invoke(nameof(CreateNavMeshForBoarding), 0.6f); // Delay to ensure doors are fully open
                        }
                    }
                    else if (busSpeed > 0 && startedBoarding && !endedBoarding) // leaving the station
                    {
                        endedBoarding = true;
                        busDoorManager.CloseDoorsBasedOnScene();
                        RemoveNavMeshForBoarding();
                    }
                }
                else if (startedBoarding && endedBoarding)
                {
                    startedBoarding = false;
                    endedBoarding = false;
                    frameCounter = 0; // Reset frame counter
                }
            }
        }

        // Triggers
        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag(busWheelTag))
            {
                Debug.Log("Bus has entered the station area.");
                busInStation = true;
                bus = other.transform.root.gameObject;
                Debug.Log("BusFloor position: " + bus.transform.Find("BusFloor").position);
                Debug.Log("Bus between doors collider position: " + bus.transform.Find("BetweenDoors").position);

                // Get the lower bound (minimum y) of the BetweenDoors collider in world space
                Collider betweenDoorsCollider = bus.transform.Find("BetweenDoors").GetComponent<Collider>();
                Vector3 betweenDoorsMin = betweenDoorsCollider.bounds.min;
                Debug.Log("BetweenDoors collider lower bound (world): " + betweenDoorsMin);

                Debug.Log("Bus station floor plane position: " + this.transform.Find("Plane").position);

                Collider frontRightDoorCollider = bus.transform.Find("FrontRightDoor").GetComponent<Collider>();
                Debug.Log("FrontRightDoor collider lower bound (world): " + frontRightDoorCollider.bounds.min);

                Collider frontRightRampCollider = bus.transform.Find("FrontRightRamp").GetComponent<Collider>();
                Debug.Log("FrontRamp collider lower bound (world): " + frontRightRampCollider.bounds.min);
                Debug.Log("FronttRamp collider upper bound (world): " + frontRightRampCollider.bounds.max);

            }
            else if (other.CompareTag("Player"))
            {
                Debug.Log("A passenger has entered the station area.");
                // Handle passenger logic here if needed
                var controller = other.GetComponent<ThirdPersonController>();
                if (controller != null)
                {
                    // Pass the target boarding position inside the bus
                    // Find the "BusFloor" child and use its center position
                    Transform busFloor = bus.transform.Find("BusFloor");
                    controller.BeginBoarding(busFloor.position);
                }
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.CompareTag(busWheelTag))
            {
                Debug.Log("Bus has exited the station area.");
                bus = null;
                busInStation = false;
                // RemoveNavMeshForBoarding(); // Safety: remove if not already removed
            }
        }


        void CreateNavMeshForBoarding()
        {
            // Create a new GameObject for the NavMeshSurface
            tempNavMeshSurfaceObj = new GameObject("TempNavMeshSurface");
            tempNavMeshSurface = tempNavMeshSurfaceObj.AddComponent<NavMeshSurface>();

            // Set Use Geometry to Physics Colliders
            tempNavMeshSurface.useGeometry = NavMeshCollectGeometry.PhysicsColliders; // PhysicsColliders doens't work
            Debug.Log("bus nav mesh geometry " + tempNavMeshSurface.useGeometry.ToString());

            // Set Object Collection to All (or Volume if you want to restrict area)
            tempNavMeshSurface.collectObjects = CollectObjects.All;

            // Set Include Layers to only your BusNavMesh layer
            tempNavMeshSurface.layerMask = LayerMask.GetMask("BusNavMesh");

            // Bake the navmesh
            tempNavMeshSurface.BuildNavMesh();
        }

        void RemoveNavMeshForBoarding()
        {
            if (tempNavMeshSurface != null)
            {
                tempNavMeshSurface.RemoveData(); // Remove the baked NavMesh
                Destroy(tempNavMeshSurfaceObj);  // Destroy the GameObject
                tempNavMeshSurface = null;
                tempNavMeshSurfaceObj = null;
            }
        }

        // Alternative approach using NavMeshSurface component

        // Create NavMesh for boarding
        // void CreateNavMeshForBoarding()
        // {
        //     if (tempNavMeshParent != null)
        //         return; // Already created

        //     // Find all relevant transforms
        //     Transform busStation = this.transform;
        //     Transform busFloor = bus.transform.Find("BusFloor");
        //     Transform frontRamp = bus.transform.Find("FrontRamp");
        //     Transform rearRamp = bus.transform.Find("RearRamp");

        //     navMeshObjects = new Transform[] { busStation, busFloor, frontRamp, rearRamp };

        //     // check which ramps should not be added because of the mesh


        //     originalParents = new Transform[navMeshObjects.Length];

        //     // Store original parents
        //     for (int i = 0; i < navMeshObjects.Length; i++)
        //         originalParents[i] = navMeshObjects[i].parent;

        //     // Create temporary parent
        //     tempNavMeshParent = new GameObject("TempNavMeshParent");
        //     foreach (var obj in navMeshObjects)
        //         obj.parent = tempNavMeshParent.transform;

        //     // Add NavMeshSurface and bake
        //     tempNavMeshSurface = tempNavMeshParent.AddComponent<NavMeshSurface>();
        //     tempNavMeshSurface.collectObjects = CollectObjects.Children;
        //     tempNavMeshSurface.BuildNavMesh();

        //     // Restore original parents
        //     for (int i = 0; i < navMeshObjects.Length; i++)
        //         navMeshObjects[i].parent = originalParents[i];
        // }

        // // Remove NavMesh and clean up
        // void RemoveNavMeshForBoarding()
        // {
        //     if (tempNavMeshSurface != null)
        //     {
        //         tempNavMeshSurface.RemoveData();
        //         tempNavMeshSurface = null;
        //     }
        //     if (tempNavMeshParent != null)
        //     {
        //         Destroy(tempNavMeshParent);
        //         tempNavMeshParent = null;
        //     }
        //     navMeshObjects = null;
        //     originalParents = null;
        // }
    }
}
