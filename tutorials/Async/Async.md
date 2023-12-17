# Asynchronous Motion with WPID

The WPID library utilizes threads to perform motion on mechanisms at the same time. This allows the robot to move its chassis and a mechanisms simultaneously to speed up your routine. Its extremely useful when you need the robot to move to a point, and then lift up a claw to grab an object. Instead of first moving the robot then lifting up the claw, we can do them at the exact same time!

## Using Async Movement with the Mechanism class

Doing asyncronous motion with WPID is very easy and only requires the use of two functions. The first is an asynchronous variant of either the `moveRelative` or `moveAbsolute` functions aptly named `moveRelativeAsync` and `moveAbsoluteAsync`.
The second function is a blocking function that waits until the movement has settled, also conviently named `waitUntilSettled`.



Here is an example of how to use those functions on a mechanism controlling a fourbar:

```cpp
fourbar.moveRelativeAsync(90, 50);
// do some other stuff here
fourbar.waitUntilSettled();
// stuff down here is blocked
```

The first call `fourbar.moveRelativeAsync(90, 50)` moves the fourbar up by 90 degrees at 50% speed, just like how the synchronous counterpart would. The difference is that anything called after this function will also run, as the async functions are running in the background. Once `fourbar.waitUntilSettled()` is called, the program holds at this line, and effectively stops the program until the fourbar has finished its movement. Once it does, the program continues as normal. 

## Using Async Movement with the Chassis classes

Async motion with the chassis is just as easy as the mechanisms, since mechanisms are utilized in the chassis. The difference is the syncrhonous chassis movement functions have asynchronous counterparts. 

For the Tank class we have:

- `void straightAsync(float distance, int max_speed)`
- `void turnAsync(float target_angle, int max_speed)`

and for HDrive, along with straight and turn, there's:

- `strafeAsync(float distance, int max_speed)`
- `diagonalAsync(float straight_distance, float strafe_distance, int straight_max_speed)`

The chassis also have a `waitUntilSettled` function that does exactly the same thing as the mechanism variant.

Here is an example of an HDrive chassis using these functions:

```cpp
hdrive.strafeAsync(24, 40);
// do some stuff here 
hdrive.waitUntilSettled();
// stuff down here is blocked
```

Just like the mechanism, each call to an async function runs the movement in the background, allowing the program to continue until the `waitUntilSettled` funciton is called. It waits for all motors connected to the drive train to finish moving and then the program continues. 

## Using Both Chassis and Mechanism Together

Doing async movement can complicate a routine. You have to make sure that each call does not interfere with any previous movement. For example, if you make two calls to the mechanism without letting it settle, it can cause the mechanism to move under the influence of two PID loops which will unsettle the robot and potentially ruin your routine. This is why using the `waitUntilSettled` function is always recommended before you make a new call on the same object. 

Here is a larger example of an autonomous routine using both synchronous and asynchronous movement with an HDrive chassis and a Mechanism called fourbar.

```cpp
// auton.cpp
void auton(){
    chassis.straight(24, 50);
    chassis.straightAsync(-24, 50);
    fourbar.moveAbsolute(60, 50);
    chassis.waitUntilSettled();
    fourbar.moveRelativeAsync(20, 50);
    chassis.turn(90, 25);
    fourbar.waitUntilSettled();
    fourbar.moveAbsolute(0, 100);
}
```

In this example, the chassis moves forwards 24 inches, then reverses asynchronously, allowing the fourbar to move to 60 degrees. We wait until the chassis reaches its target, and then move the fourbar asynchronously to 80 degrees allowing the chassis to turn 90 degrees. Then we wait for the fourbar to finish, and finally bring it back down to 0 degrees. 

With only a few lines of code, we were able to make a relatively simple and quick autonomous that does not get stuck waiting for other components of the robot to finish its movement. This speeds things up drastically, especially when there is only a few seconds for the entire routine.