# WPID Chassis

The chassis class in the WPID library defines a chassis as the drivetrain of a robot. The chassis is necessary for any driving actions that a robot must take on the field. Our library's implementation of chassis consists of a hierarchy of different common chassis types used in the VRC setting: Tank Drive (Skid Steer) and H-Drive.

# Chassis Hierarchy

The entire chassis hierarchy is built off of the mechanism class, where each separate group of motors in the chassis is its own mechanism. This means that the mechanism function `spinToTarget(void* args)` internally implements PID for any chassis motion. Before continuing with this tutorial, read through the mechanism tutorial to get a better idea of how mechanism functions operate.

WPID's chassis hierarchy includes one abstract class Chassis, its subclass Tank, and a Tank subclass HDrive. The abstract Chassis.h class contains all function headers and data fields that are shared between the chassis types we consider. The Tank.h/Tank.cpp classes declare and implement each of the functions from the Chassis.h class. The Tank class also inherits all of the data fields contained in Chassis.h. The HDrive.h/HDrive.cpp classes declare and implement each of the functions from the Chassis.h class plus new functions that relate to the extra center motor_group. Likewise, the HDrive class inherits all of the data field contained in Chassis.h but adds new data fields that relate to the center motor_group.

```
Chassis.h
    Tank.h/Tank.cpp
        HDrive.h/HDrive.cpp
```
> Visualization of the WPID chassis hierarchy

---
# How to Set Up a Chassis.h Subclass

In order to use either the Tank or HDrive classes, it is necessary to understand the required setup and available movement functions. Let us first consider the Tank class:

The first thing to do to setup your Tank class is to navigate to your init.h and init.cpp classes to instantiate and initialize your chassis. In init.h, instantiate and name an instance of the Tank class using the extern keyword.

```cpp
extern Tank chassis;
```
> Instantiation example of a Tank chassis in init.h

Next, in your init.cpp file, call the Tank constructor and pass it the appropriate arguments to initialize the Tank instance. The first parameter for the Tank constructor is the `track_width` of your robot (distance between the centers/contact patches of the front two wheels). The next parameter is the `wheel_radius` of the chassis wheels. The next two parameters are of vex::motor_group type, representing the `left` and `right` side of the robot. It is possible to have a motor_group consist of only one motor, so 2wd and 4wd are both possible using the WPID Tank class. Remember to initialize each motor individually and add it to the motor_group before calling the Tank constructor. The last parameter is the `drive_gear_ratio`. If you do not know what a gear ratio is, check out the mechanism tutorial for more resources.

```cpp
Tank(float track_width, float wheel_radius, vex::motor_group left, vex::motor_group right, float drive_gear_ratio);
```
> Tank constructor

Below is an example of a Tank setup with a 12.5" track_width, a 1.625" wheel_radius, a 4wd system, and a default drive_gear_ratio. Set drive_gear_ratio to 1 if you have no external gearing on the drivetrain.

```cpp
//Left individual motors
motor leftFront = motor(PORT17, ratio18_1, false);
motor leftBack = motor(PORT18, ratio18_1, false);

//Right individual motors
motor rightFront = motor(PORT19, ratio18_1, true);
motor rightBack = motor(PORT20, ratio18_1, true);

//Left motor_group
motor_group leftGroup = motor_group(leftFront, leftBack);

//Right motor_group
motor_group rightGroup = motor_group(rightFront, rightBack);

Tank chassis = Tank(12.5, 1.625, leftGroup, rightGroup, 1);
```
> Example Tank initialization in init.cpp

The HDrive instantiation and initialization is identical to the Tank set up, but involves one more center motor_group:

```cpp
extern HDrive chassis;
```
> Instantiation example of an HDrive chassis in init.h

Note that the HDrive constructor contains one more wheel radius and one more motor_group than the Tank constructor. The `center_wheel_radius` parameter allows for any HDrive chassis that has different size middle wheels.

```cpp
HDrive(float track_width, float wheel_radius, float center_wheel_radius, vex::motor_group left, vex::motor_group right, vex::motor_group center, float drive_gear_ratio);
```
> HDrive constructor

Below is an example of an HDrive setup with a 12.5" track_width, a 1.625" wheel_radius, a 2" center_wheel_radius, a 4wd system (+ center wheel), and a default drive_gear_ratio.

```cpp
//Left individual motors
motor leftFront = motor(PORT17, ratio18_1, false);
motor leftBack = motor(PORT18, ratio18_1, false);

//Right individual motors
motor rightFront = motor(PORT19, ratio18_1, true);
motor rightBack = motor(PORT20, ratio18_1, true);

//Center individual motor
motor center = motor(PORT16, ratio18_1, true);

//Left motor_group
motor_group leftGroup = motor_group(leftFront, leftBack);

//Right motor_group
motor_group rightGroup = motor_group(rightFront, rightBack);

//Center motor_group
motor_group centerGroup = motor_group(center);

HDrive chassis = HDrive(12.5, 1.625, 2, leftGroup, rightGroup, centerGroup, 1);
```
> Example HDrive initialization in init.cpp

---
