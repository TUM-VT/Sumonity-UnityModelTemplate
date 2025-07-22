using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace tum_bus_controller
{

    public enum ScenarioEnum
    {
        AllDoorsOpen,
        OnlyOneDoorOpen,
        FrontDoorsOpen
    }

    public class ScenarioManager : MonoBehaviour
    {

        [Header("Door Scenario Selection")]
        [SerializeField] private ScenarioEnum scenario;

        public ScenarioEnum GetScenario()
        {
            return scenario;
        }
    }
}


