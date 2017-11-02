#ifndef SIMCONNECTINTERFACE_H
#define SIMCONNECTINTERFACE_H

/* This file contains all required enums, constants and structures required to
exchange data with FS through the SimConnect interface.
*/

/* FSX Simulation phyiscs frames rate per second: this is actually variable
but we don't currently make use of it.
*/
const double SIM_UPDATE_RATE = 30;

/* Input Group IDs */
enum GROUP_ID {
  GROUP_0,
};

enum INPUT_ID {
  INPUT_XAXIS,
};

enum EVENT_ID {
  EVENT_SIM_START,
  EVENT_XAXIS,
};

enum REQUEST_ID {
  REQUEST_AIRCRAFT_POSITION,
  REQUEST_AUTOPILOT_HEADING,
};

/* Structure Definition ID's */
enum DATA_DEFINE_ID {
  DEFINITION_AIRCRAFT_POSITION,
  DEFINITION_AUTOPILOT_SELECTED_HEADING,
  DEFINITION_AIRCRAFT_ROLL_CONTROL,
};

/* Start of Structure Definitions */

/* Struct used to get position from simulator */
struct structAircraftPosition {
  double bank_rad; // bank angle in radians
  double heading; // true aircraft heading in radians
};

/* Struct used to get the heading from the autopilot panel */
struct structAutopilotSelectedHeading {
  double heading; // set heading in degrees
};

/* Struct used to send controls to the simulator */
struct structAircraftRollControl {
  double aileronDeflect = 0;
};

#endif


