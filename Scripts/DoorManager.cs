using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.AI.Navigation;

namespace tum_bus_controller
{
    public class DoorManager : MonoBehaviour
    {
        [Header("Door Animators")]
        [SerializeField] private Animator frontLeftAnimator;
        [SerializeField] private Animator frontRightAnimator;
        [SerializeField] private Animator rearLeftAnimator;
        [SerializeField] private Animator rearRightAnimator;
        private ScenarioEnum scenario;

        private const string boolParamFrontLeftString = "isFrontLeftDoorOpen";
        private const string boolParamFrontRight = "isFrontRightDoorOpen";
        private const string boolParamRearLeft = "isRearLeftDoorOpen";
        private const string boolParamRearRight = "isRearRightDoorOpen";

        private GameObject frontLeftRamp;
        private GameObject frontRightRamp;
        private GameObject rearLeftRamp;
        private GameObject rearRightRamp;

        void Start()
        {
            // Initialize door ramps
            frontLeftRamp = GameObject.Find("FrontLeftRamp");
            frontRightRamp = GameObject.Find("FrontRightRamp");
            rearLeftRamp = GameObject.Find("RearLeftRamp");
            rearRightRamp = GameObject.Find("RearRightRamp");

            // Set all doors to closed initially
            SetAllDoors(false);
            scenario = GameObject.Find("ScenarioManager").GetComponent<ScenarioManager>().GetScenario();
            Debug.Log("DoorManager initialized with scenario: " + scenario);



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
            frontLeftRamp.SetActive(isOpen);
            frontRightRamp.SetActive(isOpen);
        }

        void SetRearDoors(bool isOpen)
        {
            rearLeftAnimator.SetBool(boolParamRearLeft, isOpen);
            rearRightAnimator.SetBool(boolParamRearRight, isOpen);
            rearLeftRamp.SetActive(isOpen);
            rearRightRamp.SetActive(isOpen);
        }

        void SetLeftDoors(bool isOpen)
        {
            frontLeftAnimator.SetBool(boolParamFrontLeftString, isOpen);
            rearLeftAnimator.SetBool(boolParamRearLeft, isOpen);
            frontLeftRamp.SetActive(isOpen);
            rearLeftRamp.SetActive(isOpen);
        }

        void SetRightDoors(bool isOpen)
        {
            frontRightAnimator.SetBool(boolParamFrontRight, isOpen);
            rearRightAnimator.SetBool(boolParamRearRight, isOpen);
            frontRightRamp.SetActive(isOpen);
            rearRightRamp.SetActive(isOpen);
        }


    }
}