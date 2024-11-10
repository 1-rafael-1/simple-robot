# simple-robot
A very simple robot with HC-SR04 distance sensor and autonomous as well as remote controlled movement written in Rust

![Robot Top View](./media/top-view.jpg)
*Top view of the simple-robot showing the main components and assembly*

## See it in action!

Check out the robot navigating autonomously:

https://github.com/1-rafael-1/simple-robot/raw/main/media/autonomous-operation.mp4

## Work in Progress!

This is a WIP for my 9yo son, who wants me to build a robot for him.

![Robot Side View](./media/right-view.jpg)
*Side view showing the robot's profile and sensor placement*

The thing will be a very simple machine. I am aiming at the following:

+ Use a simple design, that is easily printed. For now I have settled on <https://www.thingiverse.com/thing:1582398>.
+ Use a Raspberry Pi Pico 2 for controller. Why on earth use such a powerful controller? -> Because I need an excuse to tinker with that.
+ Reuse the overall architecture from <https://github.com/1-rafael-1/pi-pico-alarmclock-rust>
+ Use much of the electronics components, some overly eager guy in my household bought a while ago in too large quantities. That is especially:
    + Li-Ion 18650 batteries and holders
    + step-up converters to 5V
    + HC-SR04 distance sensor
    + ...but of course this is ample opportunity to buy even more stuff... so a motor driver module, motors, and a simple RC unit and receiver... not to mention one absolutely has to buy two Pi Picos to make one device.

These are the features it should have in the end:

+ Actually be built and soldered and arrive at general working condition. Very important feature!
+ Manual mode: Be able to drive the bot as an RC car, when in manual mode.
    + As a bonus with collision control, so auto stop when nearing an obstacle and while too close refusing to drive in forward direction so an 8yo user does the least possible damage to living room furniture.
+ Autonomous mode: Be able to drive around for no apparent reason until either battery or user patience runs out. 
    + Have a simple drive control that when approaching an obstacle stops at a specified distance, makes a random turn and drives on. Much as first gen autonomous vacuum cleaners did.

## What I want to learn

This is what I hope to learn about in this project:

+ Never did anything with motors and motor control, so hoping to learn about how to do that. So handle acceleration, deceleration, calibrating to drive on a straight line, turning on the spot and turning on the move, .... all of that likely less easy than it looks.
+ Time distance measurements on a moving platform, feed the measured data into the overall flow and time reaction to that and acquisition of fresh data to achieve smooth continuous operation.
+ Use a Raspberry Pi Pico 2 with Rust. At this point in time the probe-rs team is hard at work getting the rp2350 supported. Before that happens it will be not convenient to flash/debug the thing. Fingers crossed that happens before I arrive at actually doing much...

## And stuff I learned

+ Making an async driver: <https://crates.io/crates/hcsr04_async>, which was fun. See <https://github.com/1-rafael-1/hcsr04_async>
    + Using an oscilloscope to troubleshoot
    + And learn what async should not do :-)
+ Filter sensor data with a moving median, make a package for embedded: <https://crates.io/crates/moving_median>. See <https://github.com/1-rafael-1/moving_median>
+ Kids are impatient... "Is the robot done yet?" - maybe next time I'll not make a toy...
