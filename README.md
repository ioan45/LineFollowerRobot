# Line follower robot project

#### Task:

Build a robot which resembles a little car and uses infrared light sensors to detect a black line on a white surface and continues following it by giving appropriate power to each of the two motors using PID control mechanism. The physical robot is built using the components of the given kit. The robot has to finish the given circuit in less than 20 seconds for maximum grade.

#### Components:
  * Arduino Uno microcontroller board
  * Breadboard
  * 3D printed chassis
  * Ball caster
  * QTR-8A reflectance sensor array
  * DC motor (x2)
  * L293D motor driver
  * Wheel (x2)
  * LiPo battery
  * Wires

#### Notes regarding the used PID controller:
  * The robot actually uses just the PD part of the mechanism.
  * In addition to the PD part, the robot turning speed keeps decreasing as the error keeps scaling in the same direction (as the error keeps decreasing or increasing). That is done by using a counter for the number of consecutive loop iterations in which the error scaling is the same. On that counter is applied an empirical multiplier and the resulting value is used to decrease the turning speed.

#### Repo of the colleague: https://github.com/Mihai-Lazar26/LineFolower

#### The showcase video can be found [here](https://www.youtube.com/watch?v=JXPrWjsa9Xg).

#### Photo of the setup:
<img src="/line_follower/photo.jpeg?raw=true" width=25% height=25% />

