# The WPID Mechanism Class

The Mechanism class is the basis of how the WPID library functions. It is essentially a replacement or wrapper class for `vex::motor_group`s. This means that any collection of motors you have that spin in the same direction and speed can be used in the Mechanism class, and have access to the PID algorithm as well as other utilities. In this tutorial, you will learn how create a Mechanism object to control your motors, along with all the functions associated with the Mechanism class.

---

## Creating a Mechanism

To get started, you first want to include the `Mechanism.h` header file. You can also choose to include the `wpid.h` file instead to access the entire library such as the conversion and logging classes.
> Make sure you also include the vex header files to access the VEX API

```cpp
#include "v5_vcs.h"
#include "WPID/wpid.h" // the entire library
```

In this tutorial, we are going to use a fourbar as an example. There are 4 total motors used on the lift, so lets make those. There are also some external gears, resulting in a 2:1 gear ratio, which simplifies to 2. This means the motors have to spin twice as far to make the output spin once.

> For a tutorial on how gear ratios work check out [this article](https://science.howstuffworks.com/transport/engines-equipment/gear-ratio.htm) by *how stuff works*

### Initializing the Motors

Now lets initialize the motors used in the fourbar, as well as add them to a `vex::motor_group`.

```cpp
#include "v5_vcs.h"
#include "WPID/wpid.h"

motor liftA = motor(PORT1, ratio18_1, false);
motor liftB = motor(PORT2, ratio18_1, false);
motor liftC = motor(PORT3, ratio18_1, true);
motor liftD = motor(PORT4, ratio18_1, true);

motor_group liftGroup = motor_group(liftA, liftB, liftC, liftD);
```

The reason for adding all the motors to a motor_group is because we treat the motors as one, with one input and output. This will control all the motors equally, making them run at the same speeds.

### Initializing the Mechanism Object

Now we can initalize the Mechanism using the liftGroup and the external gear ratio. Here is the constructor:

- `Mechanism(vex::motor_group* motors, float gear_ratio)`

Now let's use the constructor to create the fourbar Mechanism.

```cpp
Mechanism* fourbar = new Mechanism(&liftGroup, 2);
```

We made a new Mechanism called `fourbar`, with the motors in the motor group, and also gave it the external gear ratio of the lift.

### Setting a PID Object to the Mechanism

Now its time to set a PID object to the Mechanism so that we can use PID to move the fourbar.

```cpp
PID liftPID = PID(1.0, 0.15, 0.02);
fourbar->setPID(liftPID);
```

> Check out our [PID tutorial](https://wpidlib.github.io/WPID-Library-Docs/tutorials/PID/pid.html) for more information about the PID class and it's attributes

We now set a PID object with some constants to the fourbar and are ready to use the fourbar in our autonomous and driver control functions.

---

## Moving the Mechanism

Now that our Mechanism has been created, we can move it during our auton and driver control periods.
> Make sure you understand c++ and how global variables work to use the Mechanism throughout your project.

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
> Check out our [Async tutorial](https://wpidlib.github.io/WPID-Library-Docs/tutorials/Async/async.html) for more information about using the asyncrhonous functions.

### Using the Movement Functions

```cpp
fourbar->spin(50);
```

The `spin()` function requires a velocity in `velocity::pct` units. This is useful for controlling the Mechanism in driver control, as it also restricts the Mechanism to a maximum and minimum position that you can set using the `setBounds()` function. Check out the next section for more information on setting bounds.

```cpp
fourbar->stop();
```

As expected, the `stop()` function stops the Mechanism using the default brakeType. You can also set the brakeType using `setBrakeType`, explained in the next section.

```cpp
fourbar->moveRelative(60, 50);
```

The `moveRelative()` function moves the Mechanism to a relative position. It checks the current position of the lift and adds the target passed in and moves the lift to that position. The second parameter is the maximum speed the motors are allowed to spin. This is in `velocity::pct` units (percent units). In this example, the fourbar is moving 60 degrees beyond its current position at 50% speed.

```cpp
fourbar->moveAbsolute(45, 25);
```

The `moveAbsolute()` function moves the Mechanism to an absolute position. This means the Mechanism will move to the target passed in regardless of how far it is from the target. The speed is also in `velocity::pct` units. In this example we move the fourbar to 45 degrees at 25% speed.

---

## Tweaking the Behavior of a Mechanism

We can also change a few things about the fourbar, such as bounds and brake types. These tweaks are helpful to use when you want to improve the consistency and behavior of your Mechanism. Here are all the avaliable setters to tweak the Mechanism.

### The Setters

There are 4 different setters to tweak the behavior of a Mechanism:

- `setBounds(float lower_bound, float upper_bound)`
- `setBrakeType(vex::brakeType)`
- `setMaxAcceleration(float max_accel)`
- `setOffset(float offset)`

### Using the Tweaks

```cpp
fourbar->setBounds(0, 90);
```

The `setBounds()` function allows us to give the lift a range that we don't want it to exceed. For this example, we don't want the lift to go below 0 degrees, or above 90. The angle is the absolute position of the lift. If no bounds are set, then calling `spin()` will not stop the motors if they travel beyond a point. This is the default behavior. All "move" functions will not let the Mechanism move beyond any bounds set, so don't worry if your math does not add up. For example, calling `fourbar->moveAbsolute(200, 25)` will not move the fourbar to 200 degrees, but instead to 90 as that is the upper bound.

```cpp
fourbar->setBrakeType(hold);
```

The `setBrakeType()` function allows us to give the Mechanism a `vex::brakeType` to use when the lift is being told to `stop()`. The default brake mode for all new Mechanisms is `coast`, but it is recommended to change to `brake` or `hold`.

```cpp
fourbar->setMaxAcceleration(5);
```

The `setMaxAcceleration()` function passes a value in `velocityUnits::pct per PID iteration` units to the Mechanism to allow it to ramp up. The way it works is every time the PID loop iterates, it increases the speed of the motors by this value. It is important to set this to a speed that allows the motors to ramp up at a slow enough rate to reduce jerky motion, but also to get up to speed fast enough before PID starts to reduce it. You can also turn off acceleration by passing in -1. In this example, every iteration of the PID loop will cause the motors to increase their velocity by 5%. Changing the PID delay time will change how frequent this increase occurs.

```cpp
fourbar->setOffset(2);
```

Using `setOffset()` allows us to bump up or down the targets of every movement if there is a consistent amount of error that occurs regardless of your targets or speeds. For example, if the Mechanism is always 2 degrees off its target, you can pass in 2, and it will bump up every target by 2 degrees. Negative values also work if the Mechanism is overshooting its targets.

---

## Other Useful Functions

There are two other functions avaliable, which include:

- `float getPosition(vex::rotationUnits units)`
- `void resetPosition()`

```cpp
float currentPosition = fourbar->getPosition(deg);
```

The `getPosition()` function gets the current position of the Mechanism in the units passed in. This is useful for keeping track of the position of the Mechanism. The gear ratio of the Mechanism scales the output of this function accordingly. In this example, we set the output of the `getPosition()` function to a local variable called `currentPosition` that we can reference later.

```cpp
fourbar->resetPosition();
```

The `resetPosition()` function resets the current position of the Mechanism to 0, so wherever the Mechanism currently lies, that position will now become the 0 position. This is especially useful if your Mechanism uses a hardstop, and combining a physical stopping point along with resetting the position in your program can improve consistency.

## Example

Here is the full example of how we can create a Mechanism in our initialization file, and use it across the entire project.
> This example includes the use of the `extern` keyword which is essential to use for creating global variables, as well as header guards. If you don't know what these are, I recommend learning about them as they are essential for C++ projects.
  
```cpp
// initialize.h
#ifndef INITIALIZE_H
#define INITIALIZE_H

#include "v5_vcs.h"
#include "WPID/wpid.h"

using namespace vex;
using namespace wpid;

extern Mechanism* fourbar;

#endif // INITIALIZE_H
```

```cpp
// initialize.cpp
#include "initialize.h"

// the lift motors
motor liftA = motor(PORT1, ratio18_1, false);
motor liftB = motor(PORT2, ratio18_1, false);
motor liftC = motor(PORT3, ratio18_1, true);
motor liftD = motor(PORT4, ratio18_1, true);

// the lift motor group
motor_group liftGroup = motor_group(liftA, liftB, liftC, liftD);

void initialize(){
  // instantiate the fourbar Mechanism
  fourbar = new Mechanism(liftGroup, 2);

  // tweak some attributes
  fourbar->setBounds(0, 90);
  fourbar->setMaxAcceleration(5);
  fourbar->setBrakeType(hold);
  fourbar->setOffset(2);

  // add a PID object to the fourbar
  PID liftPID = PID(1.0, 0.15, 0.02);
  fourbar->setPID(liftPID);
}
```

```cpp
// main.cpp
#include "initialize.h"

int main(){
  initialize(); // initialize the fourbar
  fourbar->moveAbsolute(70, 75); // move the fourbar to 70 degrees at 75% speed
  fourbar->moveRelative(20, 45); // move the fourbar to 90 degrees at 45% speed
  fourbar->moveAbsolute(0, 100); // move the fourbar to 0  degrees at 100% speed
}
```
