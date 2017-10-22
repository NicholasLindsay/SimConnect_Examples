#ifndef SIMCONNECTSTRUCT_H
#define SIMCONNECTSTRUCT_H

/* This file contains all required enums, constants and structures required to 
   exchange data with FS through the SimConnect interface.
*/

/* FSX Simulation phyiscs frames rate per second */
const double SIM_UPDATE_RATE = 60;

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
};

/* Structure Definition ID's */
enum DATA_DEFINE_ID {
  DEFINITION_AIRCRAFT_ROLL,
  DEFINITION_AIRCRAFT_POSITION,
  DEFINITION_AIRCRAFT_ROLL_CONTROL,
};

/* Start of Structure Definitions */

/* Struct used to get position from simulator */
struct structAircraftPosition {
  double bank_rad; // bank angle in radians
  double rotation_vel_x_rad_s; // roll rate in radians/second
};

/* Struct used to send controls to the simulator */
struct structAircraftRollControl {
  double aileronDeflect = 0;
};

#endif


