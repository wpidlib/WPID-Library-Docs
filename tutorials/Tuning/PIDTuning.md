# How to Tune PID Objects

Properly tuning the Kp, Ki, and Kd constants is a crucial step needed to produce an efficient PID-controlled system using any PID algorithm. In the WPID library, the PID constants work in conjunction with the additional tuning features, so the tuning process is split into two parts:

1. Tune the base Kp, Ki, Kd constants
2. Enable and alter preferred tuning features

This tutorial gives a basic overview of these steps with our own examples. It is important to note that PID tuning is highly variable depending on a number of factors, so getting different constants than those used in our examples should be expected.

---
## How to Tune Kp, Ki, and Kd

When tuning Kp, Ki, and Kd constants, we recommend that you begin by setting Ki and Kd to 0. This lets you isolate the proportional element of PID control, which has the largest impact of the PID elements on any given motion. Start the Kp constant at `kp = 1` and test your robot. If you find that the robot is not making it to the set-point fast enough, or there is a lot of steady-state error (the robot is getting stuck well before the set-point), you should increase Kp. If Kp is way too high, lower it incrementally until the robot is no longer overshooting its target. It is better to have your robot slightly undershooting than overshooting at this point, as adding in the Ki constant with an overshooting P-controller will worsen that overshoot. Motion using a P-controller is generally much rougher than motion using a PI or PID controller, so do not expect to have refined motion until you add in the other constants.

```cpp
PID straight = PID(1, 0, 0);
```
> Initial tuning state

After you have determined a reasonable Kp value, add in a Ki that is 10% of your Kp. In an example case, the reasonable Kp value for our PID straight object was found to be 0.15. Thus, we would start with `ki = 0.015`. If your robot is not initially moving as fast as you would like or slows down too much near the end of the motion, increase Ki. If your robot is overshooting its target by a large margin or oscillating, decrease Ki. Once your robot is approaching its target rapidly without oscillating, you have a successful PI-controller. Having a small amount of overshoot here will actually make tuning Kd easier, so do not feel like you have to eliminate *all* overshoot with just Kp and Ki.

```cpp
PID straight = PID(0.15, 0.015, 0);
```
> Adding in initial Ki example

Finally, add in a Kd value that is smaller than your current integral value. In our example case, the reasonable Ki value for our PID straight object was found to be 0.025. We can start with `kd = 0.01` and tune the value from there. Increase kd until overshoot is eliminated from the motion. Be careful not to increase the Kd constant too much, as the derivative element is the most susceptible to fluctuations, even after being filtered. Smoother motion is achieved with a small Kd value, and we found values over `0.04` severly degrade the quality of the motion. 

```cpp
PID straight = PID(0.15, 0.025, 0.01);
```
> Adding in initial Kd example

In our example case, we determined that the best Kd value was 0.015 after running more tuning tests. Keep in mind that you should be testing the same motion throughout the tuning process to eliminate as many external factors as possible. In this example, we use the simple autonomous route `chassis.straight(24, 50)` for tuning. The final PID straight object from this example is initialized below:

```cpp
PID straight = PID(0.15, 0.025, 0.015);
```
> Tuned PID example

---
### How to Tune Additional Features

Now that you have tuned the base PID constants, you can add in the tuning features that supplement the PID algorithm in the WPID library. None of these features are required to be manually set when initializing PID objects, and some features start disabled. All of these tuning features are described further in the What is PID? tutorial, so read that tutorial beforehand to better understand why each feature is important. The tuning features and their default values are listed below:

- `Error Range` is set to a default of 2 degrees
- `Low Speed Threshold` is 2% by default
- `Max Integral` is set to a default of 100
- `Timeout` is disabled by default (set to -1)

We recommend that you first consider what error range to use with your PID object. If the task you need PID for requires a high degree of accuracy, lower the error range below 2 degrees. Keep in mind, the more you lower the error range, the longer your robot will take to complete the motion (higher settling time). If the task you need PID for is time-sensitive, increase the error range above 2 degrees. However, an error range that is set too large will sacrifice accuracy for speed. Below is an example of the PID straight object with an error range that is *too small* for our tuning route `chassis.straight(24, 50)`.

```cpp
straight.setErrorRange(0.5);
```
> Error range too large for straight motion

[GIF coming soon!]

Next we suggest that you tune the low speed threshold. Low speed threshold works directly with error range as a stopping criteria. From our experience tuning, an effective low speed threshold generally falls in the range of {1-5}. If your robot slows down too much towards the end of the PID motion, increase the low speed threshold. If the robot is stopping abruptly at the end of the PID motion, decrease the low speed threshold. Your low speed threshold is much too high if you are seeing one motor_group on the robot stop before the other (specifically seen in chassis). Below is an example of the PID straight object with a low speed threshold that is *too high* for our tuning route `chassis.straight(24, 50)`.

```cpp
straight.setLowSpeedThreshold(50);
```
> Low speed threshold too high for straight motion

![Screenshot of WPID v0.1.0](rightsidestopfirst.gif)

After tuning the stopping criteria features, you should determine whether you want to use the max integral feature. Max integral supports our library's built-in integral clamping to curb the integral build-up during PID. While not technically disabled outright, the default value for max integral is equivalent to the max speed for V5 motors. This means that there is initially no limit on the maximum integral value. As you decrease the max integral value, the integral element of PID has less overall weight on the PID calculations. From our experience tuning, an effective max integral value generally falls in the range of {5-20}. A max integral value that is too low will start to decrease the effectiveness of the integral element. Below is an example of the PID straight object with a max integral value that is *too low* for our tuning route `chassis.straight(24, 50)`.

```cpp
straight.setMaxIntegral(1);
```
> Max integral too low for straight motion

[GIF coming soon!]

The last step in tuning the additional features is to decide whether to use a timeout. Timeout is an external failsafe that overrides all other PID factors (constants and other tuning features) to stop the robot after a certain amount of time. Timeout is disabled until it is manually set, and should only be set at the very end of the tuning process. Because timeout has the ability to disrupt the PID process, setting a timeout that is too short risks nullifying your PID object's ability to properly respond to external factors. For example, if your robot's timeout is set to be 7 seconds (7000 ms) and another robot hits your robot slightly past its intended target at the 6.5 second mark, your robot will only have 0.5 seconds to correct its position. Make sure that you are not undermining PID's closed-loop control style when you set PID timeout. Below is an example of the PID straight object with a timeout value that is *too low* for our tuning route `chassis.straight(24, 50)`.

```cpp
straight.setTimeout(1000);
```
> Timeout too low for straight motion

![Gif of chassis timing out](timeout.gif)

---
## Conclusion

Now that you have an understanding of PID and the WPID library structure, you are officially ready to implement the library on your own robotic applications. Happy coding!