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

The initialization stage of the chassis also involves calls to specific setters that define additional attributes including brake type, offset, max acceleration, and your preferred measurement units. Note that only the PID objects are required to be set here; everything else is optional.

Apart from PID, offset, and measurement unit setters, the rest of the setter functions make calls to the specified mechanism functions for all motor_groups in the chassis. These functions are covered further in the Mechanism tutorial. 

The PID setter functions take PID objects as arguments and set the corresponding chassis attribute. These attributes are then used to swap the active PID object in the body of a movement function (described in the next section). With a Tank chassis, you have access to `pidStraight` and `pidTurn`. Using an HDrive chassis adds `pidStrafe` as a third PID attribute. IMPORTANT: YOU HAVE TO SET YOUR PID OBJECTS IN ORDER FOR THE ROBOT TO DISPLAY ANY KIND OF MOTION. If you call a movement function without setting the corresponding chassis PID object, the robot will not move.

```cpp
//Setting pidStraight
PID straight = PID(0.15, 0.025, 0.015);
chassis.setStraightPID(straight);

//Setting pidTurn
PID turn = PID(0.15, 0.05, 0.01);
chassis.setTurnPID(turn);

//Setting pidStrafe
PID strafe = PID(0.3, 0.67, 0.02);
chassis.setStrafePID(strafe);
```
> Setting PID objects for an HDrive chassis

Normally, the tuning features corresponding to each PID object are also set in this part of the init.cpp file. Both the What is PID? tutorial and the PID Tuning tutorial contain more information about this process.

The offset setter function is used to assign the distance (in your preferred measurement system) and number of degrees passively added to your input for each corresponding movement function. This function is meant to 'offset' any mechanical, environmental, or other external factors that may cause typical measurements to be consistently inaccurate. For example, if your robot *consistently* is 3° off of the `target_angle` you input for turns, you would set the `turn_offset` to 3. The reason for offset is to remove confusion when inputting target distances or angles. If your robot's measure of 90° is actually 87°, it is easier to set an offset of 3° and input 90° when you want the robot to turn 90°. Without offset, you would have to input 87° every time you wanted the robot to turn 90° and would have to input 42° every time you wanted the robot to turn 45°. This field should only be used to correct *consistent* margins of error. The default value for all offsets are 0.

```cpp
void setOffset(float straight, float turn, float strafe);
```
> Setter for HDrive offset

The measurement unit setter allows you to use units from the metric or US customary unit system (USCS) for any of your distance measurements. This setter automatically standardizes distances inputted in the chassis constructor after being called (track_width, wheel_radius, etc.), so it can be called before or after the chassis initialization. Every movement function also starts with a call to the Conversion class function standardize, which converts inputted units to the WPID standard inch units. The enumerated class offers six types of measurement units including {inches, feet, yards, meters, centimeters, millimeters}.

```cpp
//Setting measurement units to meters
chassis.setMeasurementUnits(Conversion::measurement::m);
```
> Example of measurement unit setter

Now that your chassis has been properly instantiated and initialized, you are ready to start trying out the different chassis movement functions.

---
# How Does WPID Move Your Chassis

Each chassis type comes with several different types of motion that are implemented in their respective C++ files. This tutorial will cover basic non-asynchronous movement options including moving straight (forwards and backwards), turning, strafing (for HDrive), and diagonal motion (for HDrive). The asynchronous movement options are explained in the Asynchronous Movement tutorial. 

The movement options shared between Tank and HDrive chassis include straight and turn motions. The straight motion option is used to cover a `distance` (in your preferred measurement units) at a specified `max_speed`. The turn movement option is used to rotate about the center of the chassis to reach a `target_angle` in degrees at a specified `max_speed`.

### Straight Movement

The WPID straight function implementation starts by translating your given `distance` input to our library's standard system of measurement: inches. After this, the function computes the equivalent distance in motor degrees. The function does this so that the mechanism `spinToTarget(void* args)` function can easily compare the target distance to the readings from the V5 motor internal encoders, which register motor degrees traveled. 

```cpp
    distance = Conversion::standardize(distance, this->measure_units);
    float target = ((distance) / wheel_circumference) * 360.0;
```
> Distance computation in straight function

Next, the straight function assigns the correct PID object, `pidStraight`, to each motor_group, since PID objects assigned to each mechanism change between motions (pidStraight vs. pidTurn). The function repeats this step with offset. Finally, the straight function passes the target distances and max speeds for each side to the Tank `spinToTarget(float left_target, float right_target, int l_max_spd, int r_max_spd)` function. This function is different from the mechanism spinToTarget function; its purpose is to trigger the mechanism movement functions to individually move each side of the robot. From this point forward, the mechanism class takes over with its own movement functions, which are explained in the Mechanism tutorial.

```cpp
    left->setPID(pidStraight.copy());
    right->setPID(pidStraight.copy());
    left->setOffset(straight_offset);
    right->setOffset(straight_offset);
    this->spinToTarget(target, target, max_speed, max_speed);
```
> Setting PID, offset, and calling Tank spinToTarget() in straight function

The Tank `spinToTarget(float left_target, float right_target, int l_max_spd, int r_max_spd)` function is called every time the chassis moves. For the Tank chassis this means moving straight and turning. The HDrive version of this function is also called when the chassis needs to strafe or move diagonal.

```cpp
void Tank::spinToTarget(float left_target, float right_target, int l_max_spd, int r_max_spd){    
    left->moveRelativeAsync(left_target, l_max_spd);
    right->moveRelativeAsync(right_target, r_max_spd);
}  
```
> Tank spinToTarget() function

### Turn Movement

The WPID turn function implementation starts by computing the equivalent distance to the specified `target_angle` in motor degrees. The function does this so that the mechanism `spinToTarget(void* args)` function can easily compare the target distance to the readings from the V5 motor internal encoders, which register motor degrees traveled.

```cpp
float target = ((track_width/2)*((float)(target_angle)*M_PI/180)/wheel_circumference)*360;
```
> Distance computation in turn function

Next, the turn function assigns the correct PID object, `pidTurn`, to each motor_group, since PID objects assigned to each mechanism change between motions (pidStraight vs. pidTurn). The function repeats this step with offset. Finally, the turn function passes the target distances and max speeds for each side to the Tank `spinToTarget(float left_target, float right_target, int l_max_spd, int r_max_spd)` function. Since this function makes the chassis turn, the target distances are the same value with opposite signs (+/-).

```cpp
    left->setPID(pidTurn.copy());
    right->setPID(pidTurn.copy());
    left->setOffset(turn_offset);
    right->setOffset(turn_offset);
    this->spinToTarget(target, -target, max_speed, max_speed);
```
> Setting PID, offset, and calling Tank spinToTarget() in turn function

Both straight and turn functions share many similarities in the way that they handle motion. The intention behind this similarity was to introduce an intuitive way to handle various types of motion following the same key structure. These similarities in structure extend to the additional HDrive movement functions as well.

### Strafe Movement

In order to describe strafe movement, first it is important to explain the minor difference in the HDrive `spinToTarget(float left_target, float right_target, float center_target, int l_max_spd, int r_max_spd, int c_max_spd);` function. Instead of only taking two distances and two max_speeds as parameters, this function requires three pairs of these values. This extra set of parameters `center_target` and `c_max_speed` takes into account the extra center motor_group on the chassis.

```cpp
void HDrive::spinToTarget(float left_target, float right_target, float center_target, int l_max_spd, int r_max_spd, int c_max_spd){
    left->moveRelativeAsync(left_target, l_max_spd);
    right->moveRelativeAsync(right_target, r_max_spd);
    center->moveRelativeAsync(center_target, c_max_spd);
}
```
> HDrive spinToTarget() function

The strafe function only has two differences from the straight function. First, the function does not need to set a PID object for the center motor_group, since this motor_group's PID object does not change between motions. Moving diagonal is a combination of strafing and straight motion, meaning that the `pidStrafe` object may be used for both types of motions. Thus, the strafing PID object only needs to be assigned during chassis initialization. Second, the function passes 0 values to spinToTarget() for the left and right distances and speeds. This change is also seen in the HDrive version of straight; straight sets 0 values for the center motor_group distances and speeds since the center motor_group does not spin during straight motion. This change is due to the difference in the base spinToTarget() HDrive function. Since the HDrive spinToTarget() function is used for strafing, straight, turn, and diagonal motion, it needs to require all six parameters regardless of the motion type.

```cpp
this->spinToTarget(0, 0, target, 0, 0, max_speed);
```
> HDrive strafe call to spinToTarget() with zeroed values

### Diagonal Movement

Diagonal movement is the most complicated motion that HDrive can perform. This function combines the straight and strafe functions to achieve motion with all three motor_groups. First, this function converts units into inches and then motor degrees like all other WPID movement functions:

```cpp
straight_distance = Conversion::standardize(straight_distance, this->measure_units);
strafe_distance = Conversion::standardize(strafe_distance, this->measure_units);
float straight_target = ((straight_distance + straight_offset) / wheel_circumference) * 360.0;
float strafe_target = ((strafe_distance + strafe_offset) / center_wheel_circumference) * 360.0;
```
> Distance computation in diagonal function

However, the diagonal function branches off from the other movement functions in its next step. The function needs to calculate the `center_max_speed` field to determine the proportion of the straight_speed that the center motor_group needs to spin to reach its target destination. The diagonal function is passed a `straight_distance`, `strafe_distance`, and `straight_max_speed`.

```cpp
void HDrive::diagonal(float straight_distance, float strafe_distance, int straight_max_speed)
```
> HDrive diagonal function header
    
The function calculates and applies the proportion of `strafe_distance`/`straight_distance` to `straight_max_speed` to obtain the center_max_speed.

```cpp
float center_max_speed = straight_max_speed*(strafe_distance / straight_distance);
```
> Center max speed calculation

Next, the diagonal function assigns the `pidStraight` object to the left and right motor_groups and passes all of the distance and max_speed arguments to the HDrive spinToTarget() function.

```cpp
left->setPID(pidStraight);
right->setPID(pidStraight);
this->spinToTarget(straight_target, straight_target, strafe_target, straight_max_speed, straight_max_speed, center_max_speed);
```
> Setting PID and calling HDrive spinToTarget()

---
# How Do YOU Move Your Chassis

Because of the framework set by the WPID library for Tank and HDrive movement functions, it is possible to move your chassis using a single function call in the auton.cpp class. Below are two examples of auton.cpp routines for a Tank and HDrive chassis respectively.

In the first routine, the Tank chassis first moves forward 24" at a `max_speed` of 50. After this, the chassis turns 90° right at a `max_speed` of 30, followed by a backwards motion of 12" at a `max_speed` of 50.

```cpp
void auton(){
  chassis.straight(24, 50);
  chassis.turn(90, 30);
  chassis.straight(-12, 50);
}
```
> Example Tank autonomous routine

In the next routine, the HDrive chassis first strafes 24" right at a `max_speed` of 40. After this, the chassis turns 90° right at a `max_speed` of 35. Next, the chassis moves diagonally backwards and left at a `straight_max_speed` of 40. Then the chassis turns 90° left at a `max_speed` of 35. Finally, the chassis moves backwards 24" at a `max_speed` of 40.

```cpp
void auton(){
  chassis->strafe(24, 40);
  chassis->turn(90, 35);
  chassis->diagonal(-24, -24, 40);
  chassis->turn(-90, 35);
  chassis->straight(-24, 40);
}
```
> Example HDrive autonomous routine

With properly tuned PID objects for straight, turn, and strafe (HDrive only), the WPID library can be used to great effect to move a chassis efficiently to a target location using various types of motion.