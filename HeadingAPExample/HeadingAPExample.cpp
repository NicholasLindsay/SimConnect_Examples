/*
The project simulates:
* A simple aircraft lateral heading controller
*/

#include <windows.h> 
#include <tchar.h> 
#include <stdio.h> 
#include <strsafe.h> 

#include "external/SimConnect.h" 

#include "common/PIDController.h"
#include "common/util.h"

#include "SimConnectInterface.h"

int     quit = 0;
HANDLE  hSimConnect = NULL;

/* Struct to hold the current status of all pilot inputs */
static struct PilotInputs {
  /* joystick axis readings are normalised into the range [-1, 1] */
  double joystickX = 0; /* last known value of joystick's x-axis */
} pilotInputs;

/* Actual aircraft status */
structAircraftPosition aircraft_status;

/* Selected autopilot heading */
structAutopilotSelectedHeading ap_selected_heading;

// temp: find sim rate
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;
decltype(Clock::now()) last_time(Clock::now());
int frame_count = 0;

void setupEvents()
{
  // Set up private events
  ASSERT_SC_SUCCESS(SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_XAXIS));

  // Add private events to notification group (don't mask for now)
  ASSERT_SC_SUCCESS(SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_0, EVENT_XAXIS));

  // Set highest priority so we recieve event before ESP
  ASSERT_SC_SUCCESS(SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_0, SIMCONNECT_GROUP_PRIORITY_HIGHEST));

  // Map joystick event to this
  ASSERT_SC_SUCCESS(SimConnect_MapInputEventToClientEvent(hSimConnect, INPUT_XAXIS, "joystick:0:XAxis", EVENT_XAXIS));

  // Turn joystick events on
  ASSERT_SC_SUCCESS(SimConnect_SetInputGroupState(hSimConnect, INPUT_XAXIS, SIMCONNECT_STATE_ON));
}

void setupDatadef() {
  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AIRCRAFT_POSITION, "PLANE BANK DEGREES", "Radians"));
  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AIRCRAFT_POSITION, "PLANE HEADING DEGREES TRUE", "Radians"));

  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AUTOPILOT_SELECTED_HEADING, "AUTOPILOT HEADING LOCK DIR", "Degrees"));

  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AIRCRAFT_ROLL_CONTROL, "AILERON POSITION", "Position"));
}

void setupInitialDataRequests() {
  /* Get position information for every sim frame */
  ASSERT_SC_SUCCESS(
    SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_AIRCRAFT_POSITION,
      DEFINITION_AIRCRAFT_POSITION, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, 0)
  );

  /* Get autopilot heading only when it changes */
  ASSERT_SC_SUCCESS(
    SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_AUTOPILOT_HEADING,
      DEFINITION_AUTOPILOT_SELECTED_HEADING, SIMCONNECT_OBJECT_ID_USER,
      SIMCONNECT_PERIOD_SIM_FRAME, SIMCONNECT_DATA_REQUEST_FLAG_CHANGED)
  );

}

void UpdateControls() {
  /*
    Use PID control to generate target bank angle based on heading error
  */

  /* Required bank per error in heading */
  const double BANK_PER_DEGREE_HEADING_ERROR = radians(3);

  /* PID Controller for heading: finds target bank angle to achieve set heading,
     clamping at 33 degrees to prevent overbank
  */
  static ClampedPIDController 
    headingController(BANK_PER_DEGREE_HEADING_ERROR,0,0,-radians(20),radians(20));

  /* Error between desired and actual heading */
  double heading_error = ap_selected_heading.heading - degrees(aircraft_status.heading);

  /* Update PID controller */
  double requested_bank = headingController.Update(heading_error, 1);

  /* Dump to screen */
  printf("HEADING: Req: %lf , Act: %lf , Err = %lf, Bank = %lf\n", 
    ap_selected_heading.heading, degrees(aircraft_status.heading), 
    heading_error, degrees(requested_bank));

  /* Required aileron deflection per bank error */
  const double AILERON_DEFL_PER_BANK_ERROR = 0.08 / radians(1); // 0.08 units per degree

  /*
    PID Controller for bank angle: bank towards the requested_bank
  */
  static ClampedPIDController
    bankController(AILERON_DEFL_PER_BANK_ERROR, 0, 0, -1.0, 1.0);

  /* Error between requested and actual bank */
  double bank_error = requested_bank - -aircraft_status.bank_rad;

  /* Update PID controller */
  double aileron_defl = bankController.Update(bank_error, 1);

  /* Update FSX */
  structAircraftRollControl rollControlSettings;
  rollControlSettings.aileronDeflect = aileron_defl;
  ASSERT_SC_SUCCESS(
    SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_AIRCRAFT_ROLL_CONTROL,
      SIMCONNECT_OBJECT_ID_USER, 0, 1, sizeof(rollControlSettings), &rollControlSettings));

  /* Dump to screen */
  //printf("BANK: Req: %lf , Act: %lf , Err = %lf, Bank = %lf\n",
  //  requested_bank, aircraft_status.bank_rad, bank_error, aileron_defl);
}

void CALLBACK SC_Dispatch_Handler(SIMCONNECT_RECV* pData, DWORD cbData, void *pContext)
{
  switch (pData->dwID)
  {
  case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
  {
    SIMCONNECT_RECV_SIMOBJECT_DATA *pObjData = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(pData);

    // Position data:
    if (pObjData->dwRequestID == REQUEST_AIRCRAFT_POSITION) {
      // update aircraft status struct
      aircraft_status =
        *reinterpret_cast<structAircraftPosition*>(&pObjData->dwData);

      auto t = Clock::now();
      frame_count++;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(t - last_time).count()
           > 1000) {
        printf("FR = %d\n", frame_count);
        frame_count = 0;
        last_time = t;
      }

      UpdateControls();
    }
    // Autopilot settings
    else if (pObjData->dwDefineID == REQUEST_AUTOPILOT_HEADING) {
      // update autopilot heading struct
      ap_selected_heading = 
        *reinterpret_cast<structAutopilotSelectedHeading*>(&pObjData->dwData);
      printf("New autopilot heading: %lf\n", ap_selected_heading.heading);
    }

    break;
  }
  case SIMCONNECT_RECV_ID_EVENT:
  {
    SIMCONNECT_RECV_EVENT *evt = (SIMCONNECT_RECV_EVENT*)pData;

    switch (evt->uEventID)
    {

    case EVENT_SIM_START:
    {
      // Now sim has started, enable events
      setupEvents();
    }
    break;
    case EVENT_XAXIS:
    {
      /* raw data is unsigned, so need to convert to signed before double */
      long int joystickIn = evt->dwData;
      pilotInputs.joystickX = static_cast<double>(joystickIn) / 32768;
    }
    break;
    default:
      break;
    }
    break;
  }

  case SIMCONNECT_RECV_ID_QUIT:
  {
    quit = 1;
    break;
  }

  default:
    printf("\nReceived:%d", pData->dwID);
    break;
  }
}

void runHeadingControl()
{
  // Establish connected to FSX
  while (SimConnect_Open(&hSimConnect, "Heading Autopilot", NULL, 0, 0, 0) != S_OK);

  printf("Connected...\b");

  // Setup events
  // setupEvents();

  // Setup data definitons
  setupDatadef();

  // Setup regular requests
  setupInitialDataRequests();

  // Main loop
  while (0 == quit) {
    SimConnect_CallDispatch(hSimConnect, SC_Dispatch_Handler, NULL);
  }

  SimConnect_Close(hSimConnect);
}

int main(int argc, _TCHAR* argv[])
{
  runHeadingControl();

  return 0;
}