Demonstration video: https://www.youtube.com/watch?v=JIMlrOTUWKE&t=216s

HERMES is a rechargeable self-balancing electric skateboard, mainly for personal transportation (inspired by the Segway). It can easily balance itself on its two wheels placed along its center, so that an user can easily mount HERMES for personal transport. A wired or wireless remote controller is available for the user to control the speed and direction of HERMES, depending on the user's preference. Also, an emergency kill switch is present to turn HERMES off for safety.

The goal was to build a system that is practical and pragmatic. A system for use in daily life as well as in industries. Easy to build, low cost and environment-friendly. 

Main components:
1)  A skateboard
2)  Two 24V DC motors (salvaged from old parts)
3)  Two wheels connected to the motors using bicycle chains
4)  2 x BTS7960 motor controllers
5)  Arduino MEGA 2560 microcontroller  
6)  11.1V 800 mAh LiPo battery to power the arduino
7)  2 x 12V 5Ah lead acid batteries connected in series to power the motors
8)  MPU6050 6DOF gyroscope

How it works?
1)  A Gyrcoscope is attached to the surface of the skateboard.
2)  The gyroscope measures the angle of inclination of the skateboard on both forward and backward directions.
3)  Data is read from the gyroscope, processed by the Arduino and sent to the motors turning the wheels.
4)  The motors react to neutralize the effect of inclination of the skateboard, using a PID algorithm.
5)  Constant level surface is maintained.
6)  To drive the skateboard, the user simply has to lean forward to drive forward and lean backwards to drive backwards.
7)  With a controller, one can control direction. The user can steer left or right and trim the balancing factor forward or backward.

Theory: 
Hermes will be prevented from falling by giving acceleration to the wheels according to its inclination from the vertical. If the bot gets tilts by an angle, the wheels will accelerate in that direction. Then, the center of mass of the bot will experience a pseudo force which will apply a torque opposite to the direction of tilt.
The gyroscope is able to measure the yaw, pitch and roll of the skateboard using a library in Arduino. But since only the inclination angle of the board was needed, only the pitch was used.

Applications:
1)  A Cheap personal transportation system. It will be able to reach speeds of upto 15-20 kmph and will be able to travel about 10 km in one full charge. 
2)  As Hermes can be manually controlled by the user or automated, it can be used in transportation of hazardous and corrosive materials, even during turbulence. 
3)  Extremely useful in self-stabilization of robots.
4)  Application in laboratory and production houses.
5)  Also useful for waiters and automated catering bots in restaurants and cafes.

Limitations and improvements:
The motors used in this were salvaged from old parts and do not have enough speed. The proper motors which have preferable torque-speed characteristics were unfeasible due to budget problems. For this reason, Hermes cannot balance heavier object. But by using better and more expensive motors, this problem can easily be solved. For example, its balance can be greatly improved by using 250Watt 24V electric scooter motors, which are fast enough to balance heavy objects.

24 V power supply were needed for driving the motors. For that, LiPo batteries would have been most preferable as they have very low weight to power ratio. But, they are also very expensive. So, two 12 V Lead-acid batteries were used in series. Although they are cheap, they are very heavy, which reduced the performance of Hermes.

Also, using pneumatic tyres instead of the old wheels will greatly improve its performance. Furthermore, instead of using a skateboard, the whole chassis can be built from scratch in order to fully customize it. Also, the chain and gear system of motor-wheel drive can be replaced with only gear system, which is more costly and complex, but also more effective.

Footnote: Lead acid battery charger was not available in the market. So, one had to be made. For that, a 12V 3A transformer, a
10A bridge rectifier and some 100uF 50V capacitors were used. The 220V DC was transformed to 12V DC using a transformer and a bridge rectifier was used to rectify that to about 18V DC. 3 capacitors were used in parallel to smoothen the rectified dc. 
