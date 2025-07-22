using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.AI.Navigation;
using UnityEngine.AI;
//using tumvt.sumounity.PedestrianModel;

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
        private bool startedBoarding = false;
        private bool endedBoarding = false;
        private int frameCounter = 0;

        // Bus
        private GameObject bus;
        private float busSpeed = 0f;
        private const string busWheelTag = "BusWheel";
        private List<GameObject> boardingPassengers = new List<GameObject>();
        // private HashSet<GameObject> boardingPassengers = new HashSet<GameObject>(); // HashSet ensures that each object is tracked without duplicates

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
            if (bus != null) // bus in station
            {
                busSpeed = bus.GetComponent<BusController>().currentSpeed;
                DoorManager busDoorManager = bus.GetComponent<DoorManager>(); // should we decouple this?

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
                else if (endedBoarding) // bus is in the station and boarding is done
                {
                    ResetPedestriansToSumoVehicles();
                }
            }
            else if (startedBoarding && endedBoarding) // bus left the station
            {
                startedBoarding = false;
                endedBoarding = false;
                frameCounter = 0; // Reset frame counter
                // ResetPedestriansToSumoVehicles();
            }

        }

        // Triggers
        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag(busWheelTag))
            {
                // Debug.Log("Bus has entered the station area.");
                bus = other.transform.root.gameObject;
            }
            else if (other.CompareTag("Player"))
            {
                if (!boardingPassengers.Contains(other.gameObject))
                {
                    boardingPassengers.Add(other.gameObject);
                    Debug.Log(other.gameObject.name + " has entered the station area.");
                    //var controller = other.GetComponent<ThirdPersonController>();
                    //if (controller != null)
                    //{
                    if (bus != null) // only if we have a bus in the station
                    {
                        Transform busFloor = bus.transform.Find("BusFloor");
                        Debug.Log("BusFloor Bounds: " + busFloor.GetComponent<Collider>().bounds);
                        other.gameObject.SendMessage("BeginBoarding", busFloor.GetComponent<Collider>().bounds);

                        //controller.BeginBoarding(busFloor.position);

                    }
                    //}
                }
                
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.CompareTag(busWheelTag))
            {
                // Debug.Log("Bus has exited the station area.");
                bus = null;
            }
            else if (other.CompareTag("Player"))
            {
                boardingPassengers.Remove(other.gameObject);
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

        void ResetPedestriansToSumoVehicles()
        {
            for (int i = 0; i < boardingPassengers.Count; i++)
            {
                GameObject passenger = boardingPassengers[i];
                if (passenger != null)
                {
                    passenger.SendMessage("SetToSumoVehicle"); // sort of they miss the bus
                }
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
