				*****************************
				** Description of the exam **
				*****************************

MOTOR (34:1 Metal Gearmotor 25Dx67L mm HP 12V with 48 CPR Encoder): 

Since the in the datasheet the specified Stall Torque varied for two differet voltages(12V and 6V), the considered for this practice was 12V, thus the Stall Torque chose was 120 oz·in which convert to N·m is 0.847 N·m and this is the maxTorque put in each RotationalMotor.
The next attribute to modify was the maxVelocity, in this the formula proportionated was useful:

			      MaxVel = (RPM·2·PI)/(60)

which in this case the datasheet indicates that the RPM is equal to 290, therefore the MaxVel = 30.36.

POSITION SENSOR: 

For the position sensor I based on the formula provided:

			   resolution = 2·PI/(countersPerRevolution)

Therefore, the resolution put in PositionSensor attribute was 0.003848, the noise part was left in 0.

DISTANCE SENSOR (VCNL4040):

The distance sensors contain a shape of a cube of dimension 0.01 x 0.01 x 0.01. In the lookupTable attribute basing on the datasheet, the sensor to reproduce is a proximity sensor that operates with a range from 0mm to 200mm, therefore in the lookupTable the the values put were: x = 0, y = 0, z = 0. The datasheet also states that the resolution of the sensor is of 16-bit, for instance using the formula given in class:

				resolution = 2^(bit) - 1
Giving as result resolution = 65535 which will be put together with the maximum distance that the sensor can read (0.2m). 

				x = 0    y = 0     z = 0
				x = 0.2  y = 65535 z = 0


This lookupTable for both distace sensor.In this way sensing from 0 (the start of the sensor) to the end of it.

Also, a function was created, this in order to "convert" the bits to centimeters, and so the functions returns a value in bits, this with the intention of justa giving the parameter, this was possible thanks to a three rule. This also was helpful for the point where is requiered to stop and move the robot at 17 cm. So that the three rule is like this:

			bitsToCentimeters = (desiredCentimeters · 65535 bits)/(20 cm)

In this case for 17cm the result was 55704.75 bits, however the functions thanks to the rule of three can compute any bit at any centimeter. 


JUSTIFICATION: 

The car now turns on in the same position, I've fixed the constant move with a counter for the left and right distance sensor, I was having troubles with the movement to the right and the left, however I've found the perfect combination so that it moves to the right and left correctly.
Sometimes the robot crashes with some obstacles since the sensors are not able to detect them and the robot may get there just a moment and there it goes out there, maybe adjusting the DISTANCE_OBSTACLE Macros to a higher value would correct this and therefore the robot will be able to rotate even before the 17cm (19 was the best).  

INSTRUCTIONS: 

The robot starts 1 meter apart from the 0,0,0 point respect to the floor, then dodge the four obstacles and the goes straight to the wall which also dodge with going to the left and continue around the wall indefinetely. 

NOTE: 

Might be the case that the at the beginning a WARNING appears, this is due to a triangle shape that I am using to cover the robot, however if this shape is quit the WARNING should disappear without affecting the mobility of the robot.

I added some irrelevant mass (0.1) to each wheel and roller that is why at the beginning the bunch of WARNINGS, but it should work correctly.
 

