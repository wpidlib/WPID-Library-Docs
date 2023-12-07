# The WPID Mechanism Class
The Mechanism class is essentially a replacement or wrapper class for `vex::motor_group`s. This means that any collection of motors you have that spin in the same direction and speed can be used in the Mechanism class, and have access to the PID algorithm as well as other utilities. 

---
## Creating a Mechanism
To get started, you first want to include the `Mechanism.h` header file. You can also choose to include the `wpid.h` file instead to access the entire library such as the conversion and logging classes.
> make sure you also include the vex header files to access the vex api
```cpp
// robot.cpp
#include "v5_vcs.h"
#include "WPID/wpid.h" // the entire library
```

In this tutorial, we are going to use a fourbar as an example. There are 4 total motors used on the lift, so lets make those. There are also some external gears, resulting in a 2:1 gear ratio, which simplifies to 2. This means the motors have to spin twice as far to make the output spin once. 

> For a tutorial on how gear ratios work check out [this article](https://science.howstuffworks.com/transport/engines-equipment/gear-ratio.htm) by *how stuff works*

### Initializing the Motors
```cpp
// robot.cpp
#include "v5_vcs.h"
#include "WPID/wpid.h"

motor liftA = motor(PORT1, ratio18_1, false);
motor liftB = motor(PORT2, ratio18_1, false);
motor liftC = motor(PORT3, ratio18_1, true);
motor liftD = motor(PORT4, ratio18_1, true);

motor_group liftGroup = motor_group(liftA, liftB, liftC, liftD);
```

The reason for adding all the motors to a motor_group is because we treat the motors as one group, with one input and output. This will control all the motors equally, making them run at the same speeds. 

### Initializing the Mechanism Object
Now lets initalize the Mechanism using the liftGroup and the external gear ratio.
```cpp
Mechanism* fourbar = new Mechanism(&liftGroup, 2);
```
We made a new Mechanism called `fourbar`, with the motors in the motor group, and also gave it the external gear ratio of the lift. 

### Setting a PID Object to the Mechanism
Now its time to set a PID object to the Mechanism so that we can use PID to move the fourbar.

```cpp
PID liftPID = PID(1, .15, .02);
fourbar->setPID(liftPID);
```
> Check out our PID tutorial for more information about the PID class and it's attributes

We now set a PID object with some constants to the fourbar and are ready to use the fourbar in our autonomous and driver control functions.

---
## Moving the Mechanism
Now that our Mechanism has been created, we can move it during our auton and driver control periods.
> Make sure you understand c++ and how global variables work to use the mechanism throughout your project.

### The Movement Functions
We have 4 options to move the Mechanism synchronously:
- `spin(int velocity)`
- `stop()`
- `moveRelative(float position, float max_speed)`
- `moveAbsolute(float position, float max_speed)`

And 3 options for asynchronous movement:
- `moveRelativeAsync(float position, float max_speed)`
- `moveAbsoluteAsync(float position, float max_speed)`
- `waitUntilSettled()`

In this tutorial we are only going to touch on non asynchronous functions.
> For a more in depth tutorial on how to use the asynchronous funcitons check out the [Async tutorial here]()

### Using the Movement Functions
```cpp
fourbar->spin(50);
```
The `spin` function passes in a velocity in `velocity::pct` units. This is useful for controlling the mechanism in driver control, as it also restricts the mechanism to a maximum and minimum position that you can set using `setBounds`. Check out the next section for more information on setting bounds.

```cpp
fourbar->stop();
```
As expected, the `stop` function stops the mechanism using the default brakeType. You can also set the brakeType using `setBrakeType`, explained in the next section.

```cpp
fourbar->moveRelative(60, 50);
```
The `moveRelative` function moves the mechanism to a relative position. It checks the current position of the lift and adds the target passed in and moves the lift to that position. The second parameter is the maximum speed the motors are allowed to spin. This is in `velocity::pct` units (percent units). In this example, the fourbar is moving 60 degrees beyond its current position at 50% speed. 

```cpp
fourbar->moveAbsolute(45, 25);
```
The `moveAbsolute` function moves the mechanism to an absolute position. This means the mecnahism will move to the target passed in. The speed is also in `velocity::pct` units. In this example

---
## Tweaking the Behavior of a Mechanism
We can also change a few things about the fourbar, such as bounds and brake types. These tweaks are helpful to use when you want to improve the consistency and behavior of your Mechanism. Here is an example of using all the setters. 

### The Setters
There are 4 different setters to tweak the behavior of a Mechanism:
- `setBounds(float lower_bound, float upper_bound)`
- `setBrakeType(vex::brakeType)`
- `setMaxAcceleration(float max_accel)`
- `setOffset(float offset)`

### Using the Setter

```cpp
fourbar->setBounds(0, 90);
```
The `setBounds` function allows us to give the lift a range that we don't want it to exceed. For this example, we don't want the lift to go below 0 degrees, or above 90. The angle is the absolute position, relative to where the lift starts when the program begins. If no bounds are set, then calling `spin` will not stop the motors if they travel beyond a point. This is the default behavior. All "move" functions will not let the mechanism move beyond any bounds set, so don't worry if your math does not add up.

```cpp
fourbar->setBrakeType(hold);
```
The `setBrakeType` function allows us to give the mechanism a `vex::brakeType` to use when the lift is being told to `stop`. The default brake mode for all new mechanisms is `coast`, but it is recommended to change to `brake` or `hold`.

```cpp
fourbar->setMaxAcceleration(5);
```
The `setMaxAcceleration` function passes a value in `velocityUnits::pct` to the mechanism to allow it to ramp up. The way it works is every time the PID loop iterates, it increases the speed of the motors by this value. It is important to set this to a speed that allows the motors to ramp up at a slow enough rate to reduce jerky motion, but also to get up to speed fast enough before PID starts to reduce it. You can also turn off acceleration by passing in -1. 

```cpp
fourbar->setOffset(2);
```
Using `setOffset` allows us to bump up or down the targets of every movement if there is a consistent amount of error that occurs regardless of your targets or speeds. For example, if the mechanism is always 2 degrees off its target, you can pass in 2, and it will bump up every target by 2 degrees. Negative values also work if the mechanism is undershooting its targets. 

---
## Other Useful Functions
