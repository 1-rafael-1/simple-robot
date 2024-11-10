# simple-robot
A very simple robot with HC-SR04 distance sensor and autonomous as well as remote controlled movement written in Rust

![Robot Side View](misc/media/right.jpg)
*Side view showing the robot's profile and sensor placement*

Check out the robot navigating autonomously, download the demo video:

![Autonomous Operation](misc/media/autonomous-mode.mp4)

## What is it?

This is a hobby project for my 9yo son, who wanted me to build a robot for him. The thing is a rather simple machine. I was aiming at the following:

+ Use a chassis, that is easily printed. For now I have settled on <https://www.thingiverse.com/thing:972768>. The author of that design has moved on to more sophisticated designs, I will surely come back to those. But right now this was just as simple and versatile as I wanted it to be, really cool what people have made.
    + The HC-SR04 holder is from here: <https://www.thingiverse.com/thing:189585>. It is not a perfect fit for the chassis, but works well enough.
+ Use a Raspberry Pi Pico 2 for controller. Why on earth use such a powerful controller? -> Because I needed an excuse to tinker with that.
+ Use much of the electronics components, some overly eager guy in my household bought a while ago in too large quantities. That is especially:
    + Li-Ion 18650 batteries and holders
    + step-up converters to 5V
    + HC-SR04 distance sensor
    + ...but of course this is ample opportunity to buy even more stuff... so a motor driver module, motors, and a simple RC unit and receiver... not to mention one absolutely has to buy two Pi Picos to make one device.

## What does it do?

The robot offers two main modes of operation: manual (RC control) and autonomous. Here's what each mode can do:

### Manual Mode (RC Control)
The robot can be controlled like a remote-controlled car using four buttons (A, B, C, D). Each button has two functions depending on whether you press it briefly or hold it:

Button Control Scheme:
- Button A:
  - Quick press: Forward movement. Eveery press increases speed.
  - Hold & Release: Switch to autonomous mode.
- Button B:
  - Quick press: Turn right. Every press increases turn speed.
- Button C:
  - Quick press: Turn left. Eve press increases turn speed.
- Button D:
  - Quick press: Backward movement. Everypress increases speed.

The manual mode is pure RC control without any safety features - it's up to you to drive responsibly! Also, the ultra-chead RC sender and receiver I used are awkward to use and reception range is poor, so this is more for having done it and testing stuff.

### Autonomous Mode
In this mode, the robot becomes self-driving with the following behaviors:
- Continuously moves forward while monitoring its environment
- Uses the HC-SR04 ultrasonic sensor to measure distances (in centimeters)
- When detecting an obstacle:
  - Stops at a safe distance (currently set to avoid close encounters)
  - Executes an avoidance maneuver:
    1. Stops completely
    2. Backs up a bit
    3. Makes a random turn
    4. Decides if there is still an obstacle, if yes executes avoidance maneuver again and if no resumes forward motion.

Any quick button press will end autonomous mode.

### System Features

Battery Management:
- Continuous monitoring of battery voltage through a voltage divider
- Battery level indicated via RGB LED: The color of the LED changes from green to red as the measured voltage decreases. Right now there is no deep discharge protection, so red is 2.5V and at that point better switch it off to protect the Li-Ion.

Auto Standby:
- Automatic standby mode after 3 minutes of inactivity 
- Monitors all user interactions (button presses) to reset the inactivity timer
- In standby, motors are disabled to conserve power
- Any button press will wake the robots motors from standby

Sensor System:
- HC-SR04 ultrasonic distance sensor
- Measurements taken continuously at regular intervals
- Uses moving median filtering for more reliable distance readings (I found that on a moving and vibrating platform the measurements vary wildly sometimes)

## Stuff used



