/*
  Airbus aircraft are controlled by so-called "control laws" when operating
  in standard flight. This means that under normal conditions inputs to the
  joystick do not correspond directly to deflections of the control surfaces.

  This project simulates:
  * Horizontal side-stick corresponds directly to roll angle
  * Bank angle protection mechanisms

  See http://www.airbusdriver.net/airbus_fltlaws.htm for overview of Airbus
  control laws.
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
  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AIRCRAFT_POSITION, "ROTATION VELOCITY BODY X", "Radians per second"));

  ASSERT_SC_SUCCESS(SimConnect_AddToDataDefinition(hSimConnect, DEFINITION_AIRCRAFT_ROLL_CONTROL, "AILERON POSITION", "Position"));
}

void setupInitialDataRequests() {
  /* Get position information for every sim frame */
  ASSERT_SC_SUCCESS(
    SimConnect_RequestDataOnSimObject(hSimConnect, REQUEST_AIRCRAFT_POSITION,
      DEFINITION_AIRCRAFT_POSITION, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME, 0)
  );
}

/*
  Computes the desired roll rate, including applying protection
*/

double CalculateDesiredRollRate(double joystick_input) {
  /* Relationship between requested roll rate and joystick input */
  const double RAD_S_PER_UNIT_DEFLECTION = 0.15;
  const double& MAX_REQUESTABLE_ROLL_RATE = RAD_S_PER_UNIT_DEFLECTION;

  double desired_roll_rate_rad_s = RAD_S_PER_UNIT_DEFLECTION * joystick_input;

  /*
    Apply roll protection here: if bank angle greater than 33 degrees, and no
    pressure on sidestick, request roll rate until bank angle is 33 degrees or
    less. Always prevent bank angle exceeding 67 degrees.
  */

  /* The roll rate to command when roll protection activates */
  const double RESTORING_ROLL_RATE = radians(5); // per second

  /* The bank angle at which clamping begins to prevent overbank */
  const double BANK_CLAMPING_ANGLE = radians(60);

  /* The maximum bank angle allowed */
  const double MAX_BANK_ANGLE = radians(67);

  /* The nominal bank angle: the usual bank angle for turns */
  const double NOMINAL_BANK_ANGLE = radians(33);

  /* true if aircraft is rolling in direction of bank */
  bool isRollingBankDir = (sign(aircraft_status.bank_rad) == -sign(desired_roll_rate_rad_s));

  /* maximum bank angle is 67 degrees. to enforce this, aggressively reduce
     requested roll rate as 67 degrees is approached.
  */
  if (abs(aircraft_status.bank_rad) >= MAX_BANK_ANGLE) {
    desired_roll_rate_rad_s = RESTORING_ROLL_RATE * sign(aircraft_status.bank_rad);
  }
  else if (abs(aircraft_status.bank_rad) > BANK_CLAMPING_ANGLE && isRollingBankDir && joystick_input != 0) {
    /* linearly reduce maximum allowable roll rate as maximum bank angle is approached */
    double max_roll_rate = MAX_REQUESTABLE_ROLL_RATE + 
      (abs(aircraft_status.bank_rad) - BANK_CLAMPING_ANGLE) * 
        (0 - MAX_REQUESTABLE_ROLL_RATE) / (MAX_BANK_ANGLE - BANK_CLAMPING_ANGLE);

    /* clamp pre-computed desired_roll_rate to this value */
    if (abs(desired_roll_rate_rad_s) > max_roll_rate) {
      desired_roll_rate_rad_s = max_roll_rate * sign(desired_roll_rate_rad_s);
    }
  }
  /* if no joystick input, restore aircraft to NOMINAL_BANK_ANGLE */
  else if (abs(aircraft_status.bank_rad) > NOMINAL_BANK_ANGLE && joystick_input == 0) {
    desired_roll_rate_rad_s = RESTORING_ROLL_RATE * sign(aircraft_status.bank_rad);
  }

  return desired_roll_rate_rad_s;
}

void UpdateControls() {
  /* Relculate desired roll rate from joystick input and protections */
  double desired_roll_rate = CalculateDesiredRollRate(pilotInputs.joystickX);

  /* Use a P-only controller for roll rate */
  const double AILERON_DEFL_PER_RAD_S_ERROR = 10;
  static PIDController roll_rate_controller(AILERON_DEFL_PER_RAD_S_ERROR, 0, 0);
  auto output = roll_rate_controller.Update(
    desired_roll_rate - aircraft_status.rotation_vel_x_rad_s, SIM_UPDATE_RATE);

  /* Manually clamp controller output */
  if (output > 1) {
    output = 1;
  }
  else if (output < -1) {
    output = -1;
  }

  /* Send output to FSX */
  structAircraftRollControl rollControlSettings;
  rollControlSettings.aileronDeflect = output;
  ASSERT_SC_SUCCESS(
    SimConnect_SetDataOnSimObject(hSimConnect, DEFINITION_AIRCRAFT_ROLL_CONTROL, 
      SIMCONNECT_OBJECT_ID_USER, 0, 1, sizeof(rollControlSettings), &rollControlSettings));
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
      
      UpdateControls();
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

void runFBW()
{
  // Establish connected to FSX
  while (SimConnect_Open(&hSimConnect, "Airbus Roll Control Law", NULL, 0, 0, 0) != S_OK);

  printf("Connected...\b");

  // Setup events
  setupEvents();

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
  runFBW();

  return 0;
}