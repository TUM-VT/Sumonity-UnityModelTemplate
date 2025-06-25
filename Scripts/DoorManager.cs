using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.AI.Navigation;

public class DoorManager : MonoBehaviour
{
    [Header("Door Animators")]
    [SerializeField] private Animator frontLeftAnimator;
    [SerializeField] private Animator frontRightAnimator;
    [SerializeField] private Animator rearLeftAnimator;
    [SerializeField] private Animator rearRightAnimator;

    public enum ScenarioEnum
    {
        OnlyOneDoorOpen,
        AllDoorsOpen,
        FrontDoorsOpen
    }

    [Header("Door Scenario Selection")]
    [SerializeField] private ScenarioEnum scenario;

    private const string boolParamFrontLeftString = "isFrontLeftDoorOpen";
    private const string boolParamFrontRight = "isFrontRightDoorOpen";
    private const string boolParamRearLeft = "isRearLeftDoorOpen";
    private const string boolParamRearRight = "isRearRightDoorOpen";

    [Header("NavMesh Settings")]
    // Ensure you have a NavMeshSurface component in your scene
    [SerializeField] private NavMeshSurface navMeshSurface;

    private bool isInStation = true;
    private int stationCounter = 0;

    void Start()
    {
        // UnityEngine.AI.NavMesh.RemoveAllNavMeshData();

        // Set all doors to closed initially
        SetAllDoors(false);
        
        // Open doors based on the current scene
        // openDoorsBasedOnScene(scenario);

        // Using Invoke to delay the NavMesh baking slightly to ensure doors are fully animated
        // Invoke(nameof(BakeNavMeshAfterDoorsOpen), 0.5f);

    }

    void Update()
    {   
        // just for testing purposes
        // Debug.Log("Station Counter: " + stationCounter);
        // if (isInStation && stationCounter % 1000 == 0)
        // {
        //     // Debug.Log("In station, opening doors based on scenario: " + scenario);
        //     openDoorsBasedOnScene(scenario);
        //     isInStation = false; // Reset after opening doors
        // }
        // else if (stationCounter % 1000 == 500)
        // {
        //     SetAllDoors(false);
        // }
        // stationCounter++;
    }

    public void BakeNavMeshAfterDoorsOpen()
    {
        if(navMeshSurface!=null)
        {
            navMeshSurface.BuildNavMesh();
            // Debug.Log("NavMesh baked after doors open.");
        }
        else
        {
            Debug.LogWarning("NavMeshSurface not assigned to Door Manager!");
        }
    }

    public void openDoorsBasedOnScene()
    {
        // Debug.Log("Opening doors based on scenario: " + scenario);
        switch (scenario)
        {
            case ScenarioEnum.AllDoorsOpen:
                Debug.Log("Opening all doors for scene");
                SetAllDoors(true);
                break;

            case ScenarioEnum.FrontDoorsOpen:
                Debug.Log("Opening front doors for scene");
                SetFrontDoors(true);
                SetRearDoors(false);
                break;

            case ScenarioEnum.OnlyOneDoorOpen:
                Debug.Log("Opening one door for scene");
                SetLeftDoors(false);
                SetRightDoors(true);
                break;

            default:
                Debug.LogWarning("Unhandled scene");
                SetAllDoors(false);
                break;
        }
    }

    public void CloseDoorsBasedOnScene()
    {
        // Debug.Log("Closing doors based on scenario: " + scenario);
        switch (scenario)
        {
            case ScenarioEnum.AllDoorsOpen:
                Debug.Log("Closing all doors for scene");
                SetAllDoors(false);
                break;

            case ScenarioEnum.FrontDoorsOpen:
                Debug.Log("Closing front doors for scene");
                SetFrontDoors(false);
                SetRearDoors(false);
                break;

            case ScenarioEnum.OnlyOneDoorOpen:
                Debug.Log("Closing one door for scene");
                SetLeftDoors(false);
                SetRightDoors(false);
                break;

            default:
                Debug.LogWarning("Unhandled scene");
                SetAllDoors(false);
                break;
        }
    }

    void SetAllDoors(bool isOpen)
    {
        SetFrontDoors(isOpen);
        SetRearDoors(isOpen);
    }

    void SetFrontDoors(bool isOpen)
    {
        frontLeftAnimator.SetBool(boolParamFrontLeftString, isOpen);
        frontRightAnimator.SetBool(boolParamFrontRight, isOpen);
    }

    void SetRearDoors(bool isOpen)
    {
        rearLeftAnimator.SetBool(boolParamRearLeft, isOpen);
        rearRightAnimator.SetBool(boolParamRearRight, isOpen);
    }

    void SetLeftDoors(bool isOpen)
    {
        frontLeftAnimator.SetBool(boolParamFrontLeftString, isOpen);
        rearLeftAnimator.SetBool(boolParamRearLeft, isOpen);
    }

    void SetRightDoors(bool isOpen)
    {
        frontRightAnimator.SetBool(boolParamFrontRight, isOpen);
        rearRightAnimator.SetBool(boolParamRearRight, isOpen);
    }

}
