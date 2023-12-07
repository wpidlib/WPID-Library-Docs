# PID Implementation

Our PID implementation in the WPID library uses a basic Proportional-Integral-Derivative (PID) algorithm structure with some add-on features for more efficient tuning. This tutorial covers the basics of PID and the WPID-specific implementation of the algorithm.

# What is PID?

Proportional Integral Derivative (PID) Control is the most common method of control used in industry applications. PID is preferred professionally because the algorithm has the ability to adapt to different environments and allows for generally straightforward implementation.

# Open-loop and Closed-loop Control

When considering types of motion algorithms, the options for implementation follow one of two general approaches: open-loop or closed-loop control. In open-loop control, the algorithm takes in a set of input parameters and uses the values of these fields to compute a response. During the execution of the algorithm, no new internal or external data is introduced into the calculations. This approach allows for efficient and cost-effective implementation of an algorithm. However, this control method is not able to react to changing environments.

In closed-loop control, the algorithm takes input from the initial set of parameters and factors the system output into its calculations. These systems are typically more complex than open-loop systems, as the hardware must include a component for the collection of internal or external information. Different types of sensors can be used for the collection of external output; the type of information collected varies depending on the application. Closed-loop systems are typically more expensive due to the need for sensing components, but algorithms using this approach are more precise and accurate than open-loop control systems.

# PID VEX Applications

PID control is a type of closed-loop control system. In the VEX setting, the algorithm takes in a target distance parameter as its setpoint or goal state. The system output comes from a sensor on the robot that determines the robot’s current distance from its setpoint. This sensor can be an external camera, ultrasonic range finder, line tracker, or encoder (internal or external). In th WPID implementation, the chosen sensor for system output is the internal encoder system on the VEX V5 motors. The encoder sensor responds to motion and returns the distance that the motor has traveled. This traveled-distance is subtracted from the setpoint-distance to determine the steady-state error, referred to as error in the figure below. Then, the P, I, and D constants combine with the steady-state error to calculate the speed of the robot and the loop repeats with the new encoder output.

![PID Control Diagram](PIDFigure1.png)
> Figure of PID closed-loop control

---
# How Do P, I, and D Constants Work?

The PID controller operates using a combination of three constants: the proportional constant Kp, the integral constant Ki, and the derivative constant Kd. 

The Kp constant proportionally increases the robot’s response-speed based on steady-state error. With a higher steady-state error, the Kp constant will yield a more immediate response, however; a high Kp can cause overshoot, where the robot hits its setpoint and then continues moving.

```cpp
float error;
//error*kp;
```
> Proportional element of PID control

The Ki constant multiplies with the summation of steady-state error to move the robot incrementally towards its setpoint. Ki adds more precision to the system, so that once Kp brings the robot close to its setpoint, the Ki integration can cover the remaining distance. The combination of Kp and Ki is used in the PI control configuration of PID to quickly (Kp) and precisely (Ki) reach a setpoint.

```cpp
float integral = prev_integral + (error * (delay_time/(float)1000));
//integral*ki;
```
> Integral element of PID control

The Kd constant multiplies with the difference of steady-state error to decrease overshoot. The combination of Kp and Kd is used in the PD control configuration of PID to quickly (Kp) reach a setpoint while curbing overshoot (Kd). 

```cpp
float derivative = (error - prev_error) / (delay_time/(float)1000);
//derivative*kd;
```
> Derivative element of PID control

Using all three constants in conjunction implements the PID configuration, where the robot can quickly (Kp) and precisely (Ki) reach a setpoint while curbing overshoot (Kd). The Ki and Kd constants are not used together in a configuration, as the immediate response of Kp is crucial for an efficient implementation.

```cpp
float speed = error*kp + integral*ki + derivative*kd;
```
> PID control implementation in PID.cpp

---
# Why Does PID Need Additional Utilities?

While PID serves as an adaptable and efficient algorithm for robotic motion, the algorithm has several blindspots that can be addressed with supplemental utilities. The first implementation issue with PID occurs at the intersection of the Kp and Ki constants. When continuously integrating with Ki throughout a PID-controlled motion, the integral portion of the algorithm increases to the point where overshoot is difficult to avoid. In order to eliminate overshoot from PID motions, this continuous accumulation of steady-state error needs to be limited to specific periods of the motion.

The integral clamping and max-integral limit features within the WPID library address... 

```cpp
float integral = prev_integral + (error * (delay_time/(float)1000));
if(integral*ki > max_integral_speed) {integral = max_integral_speed/ki;}
if(integral*ki < -max_integral_speed) {integral = (-max_integral_speed)/ki;}

if(abs(speed) > abs(max_speed) && std::signbit(error) == std::signbit(speed)){
    speed -= integral*ki;
    integral = prev_integral;
    speed += integral*ki;
}
prev_integral = integral;
```
> Integral clamping and limiting in PID.cpp

Another issue with PID is the lack of a filtration feature for derivative noise. The derivative portion of PID is subject to sharp fluctuations during motion that make this piece of the algorithm difficult to implement smoothly. Finally, the PID algorithm has no feature to support smooth transitions into and out of motions. PID control supports smooth motion in the intermediate stage between the start and end of the motion, but supplemental utilities are needed to prevent sudden starting and stopping.

The low-pass derivative filtration feature within the WPI library addresses...

```cpp
float a = .7;
if(prev_error == MAXFLOAT) {prev_error = error;}
float current_estimate = (previous_estimate*a + (1-a)*(error - prev_error)); 
previous_estimate = current_estimate;
prev_error = error;
float derivative = current_estimate / (delay_time/(float)1000);
```
> Derivative filtration in PID.cpp

---
# How to Use the PID Class in a WPID Project

Setting up the PID control for a VEX project using the WPID library involves two steps. First, you have to call the PID constructor and initialize the Kp, Ki, and Kd values. Below is some sample code showing the creation of a PID object.

```cpp
//PID(float kp, float ki, float kd)
PID turnPID = new PID(0.7, 0.01, 0.5);
```
> PID constructor call example in init.cpp

Next you need to call the applicable setters to initialize values used in the additional PID features. 

```cpp
void setErrorRange(float bound);
void setDelayTime(int delay);
void setBias(int bias);
void setLowSpeedThreshold(int threshold);
void setTimeout(int timeout);
void setMaxIntegral(int max_integral);
```