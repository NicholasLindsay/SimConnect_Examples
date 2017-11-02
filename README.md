# SimConnect_Examples

This repository contains different examples of using the SimConnect API to control Microsoft Flight Simulator X (FSX) externally.

Currently, there are two examples:

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
* Run roll_example.exe, provided in this repository under `prebuilt_binaries`
* Try moving your joystick left and right. Also try letting go of the joystick and have a look at the response of the aircraft. You may notice that the aircraft seems more responsive than usual.
* Try banking more than 33 degrees and then completely letting go of the joystick. The aircraft will reduce it's bank angle until it is 33 degrees.
* Try to bank to greater than 67 degrees. You will notice that the roll rate decreases very quickly despite constant side-stick pressure around 60 degrees of bank - this is to prevent the aircraft from exceeding the 67 degree limit.

### How to build

In order to build, you will require the SimConnect SDK. Copy the file `SimConnect.lib` from the SDK into the this directory, and make sure that `SimConnect.lib` is specified in Project Properties > Linker > Command Line > Additional Options.


### Description of control algorithm

The control algorithm is heavily commented inside the source code. Have a look at the *`CalculateDesiredRollRate`* and *`UpdateControls`* functions inside `main.cpp`.

![Graph of roll rate behaviour](https://github.com/NicholasLindsay/SimConnect_Examples/blob/master/doc/AllowedRollRatesvsBank.png "Allowed Roll Rate vs Bank Angle")

Brief description:

The position of the joystick commands a roll rate directly proportional to the joystick's displacement. If the bank angle of the aircraft is less than 33 degrees, this roll rate is the setpoint for the PID controller. Otherwise:
* If the bank angle is greater than 67 degrees, the FBW system commands a high rotation rate in the opposite direction to the bank angle
* If the bank angle is between 60 and 67 degrees, the FBW system clamps the requested rotation rate if it is acting to increase the bank angle
* If the bank angle is between 33 and 60 degrees, and the joystick is neutral in the roll axis, the FBW systems commands a rotation rate acting in the opposite direction to the bank angle

See the above graph for a plot of allowed rotation rates vs bank angle.

A PID controller is used to match the aircrafts actuall roll rate to the desired value. The input to the PID controller is the difference between the desired and actual rotation rate, and the output of the PID control is a number is the required deflection of the ailerons.

By default, the dynamics of the aileron servos are not simulated and therefor the required deflection is immedietely reflected in the output. In reality, the aileron servos have a first order response with a typical time constant of 0.1s. Uncomment the indicated lines in the code to enable the simulation of this response.

### Limitations

This is a simple and incomplete example. The FBW roll system does not function correctly under conditions of extreme pitch, yaw or speed. The control surfaces sometimes move instantaneously instead of gradually (this can be solved by enabling the aileron response in the code). The program is only tuned for the default Boeing 737-800. Nonetheless it is a useful proof-of-concept.

## Lateral autopilot

This is an example of a simple nested PID control algorithm for the aircraft's heading.

See code for documentation.

*TODO: Improve documentation here *

