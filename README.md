# SimConnect_Examples

This repository contains different examples of using the SimConnect API to control Microsoft Flight Simulator X (FSX) externally.

Currently, there is only one example:

## Fly-By-Wire Roll

Virtually all modern commercial airlines have a fly-by-wire based control system. This means that movement of the pilot's controls are processed electronically before the control surfaces are driven. 

The example given in this repository is an Airbus-style Fly-By-Wire control for the roll. In traditional aircraft, the ailerons are connected directly to the yoke. In an Airbus, however, the pilot flies the plane using a joystick. Under normal conditions, sideways deflection of the joystick commands a constant roll rate, irrespective of airspeed and configuration. The bank angle is also limited: letting go of the sidestick beyond 33 degrees bank will cause the aircraft to reduce it's bank angle to 33 degrees, and it is impossible to bank more than 67 degrees.

See http://www.airbusdriver.net/airbus_fltlaws.htm for a description of the Airbus control laws.

Files:
* `main.cpp` - Main entry point. Majority of control processing is carried out here, as well as direct interfacing with SimConnect
* `SimConnectInterface.h` - defines all structs, constants and enums used to communicate with SimConnect
* `PIDController.h` - a generic PID controller class
* `util.h` - Provides a set of useful functions and macros


### Example usage

To see this example in action:
* Start FSX
* Choose the Default B737-800
* Take off and achieve stable flight
* Run roll_example.exe, provided in this reposity under `prebuilt_binaries`
* Try moving your joystick left and right. Also try letting go of the joystick and have a look at the response of the aircraft. You may notice that the aircraft seems more responsive than usual.
* Try banking more than 33 degrees and then completely letting go of the joystick. The aircraft will reduce it's bank angle until it is 33 degrees.
* Try to bank to greater than 67 degrees. You will notice that the roll rate decreases very quickly despite constant side-stick pressure around 60 degrees of bank - this is to prevent the aircraft from exceeding the 67 degree limit.

### Description of control algorithm

The control algorithm is heavily commented inside the source code. Have a look at the *`CalculateDesiredRollRate`* and *`UpdateControls`* functions inside `main.cpp`.

### Limitations

This is a simple and incomplete example. The FBW roll system does not function correctly under conditions of extreme pitch, yaw or speed. The control surfaces sometimes move instantaneously instead of gradually. The program is only tuned for the default Boeing 737-800. Nonetheless it is a useful proof-of-concept.
