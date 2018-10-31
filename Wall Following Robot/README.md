# Wall Following Robot
## Description
Designed an autonomous robot car which follows along a path defined between two walls. The robot car tries to maintain itself inbetween the two walls and as close as possible towards the center. The walls are approximate 20-25 centimeters in respect to the sides of the robot car. The control of the robot car is handle inside a SysTick timer interrupt which grabs the sensor data and updates the duty cycle of the PWM. The robot car uses a pair of infrared (IR) sensors to gauge its distance relative to the walls. The IR sensors' readings are converted from analog to digital using one of the four internal ADC sequencer of the microcontroller to compute the distance. The motors are interfaced via a dual h-bridge motor driver using the a direction pin and PWM pin to control the left and right motor of the robot car. The robot car adjusts the speed of its motors via pulse width modulation (PWM) proportionally to the information received from the distances from the wall in order to avoid bumping/hitting the walls as it transverses the path. The PWM are hardware initialized and handle by changing the internal maximum counter values for their comparator. An LCD display is attached to the top of the robot which shows the distance from the walls in both direction and the current duty cycle of the PWM for both wheels.

## Components
Microcontroller: Texas Instrument Tiva 4C Launchpad - TM4C123GH6PM<br>
Parts:<br>
* 2x Sharp IR Distance Sensor (GP2Y0A21YK)<br>
* 1x L9110 Dual H-Bridge Motor Driver<br>
* 1x 10 Turn Potentiometer<br>
* 1x Nokia 5110 LCD

## Tools & Technology
Target: Texas Instrument Tiva 4C Launchpad - TM4C123GH6PM<br>
Language: Keil C<br>
Environment: Keil Uvision 4<br>
