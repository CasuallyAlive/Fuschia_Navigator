# Fuschia_Navigator

Simple overview of use/purpose.

## Description

Maze navigator rover

This is a development of a maze navigating rover. The rover utilizes three infrared sensors, two dc motors with built in encoders, two custom H-bridge motor drivers, an stm32f0 board, and several miscellaneous wires and parts. Lab protocols covered are: PID, UART, GPIO, TIMERS, INTERRUPTS. It also has a second bluetooth UART connection which can be utilized to control the rover if needed. Refer to the parts list in the embedded system final project document for more information.

The power delivery board is built using a lipo battery, a 30 A fuse, a voltage regulator paired with two capacitors. 

The navigator uses the IR sensors to drive the motors using PID. The idea is that it will go straight as long as the IR sensors are within a threshold for distance, and turn once it detects an object closing in. The turning direction is dictated by whichever of the three sensors is reading the furthest distance from the rover. This is an autonomous design, requiring little input in theory. There was an intention to provide manual override if needed, which was scrapped in the final design due to difficulties with the main part of the project. Bluetooth info will be included at the bottom of this file for anyone looking to use it. 


## Getting Started

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

FOR BLUETOOTH ONLY: Linux OS

### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
BLUETOOTH with USART

We modeled our bluetooth after the UART lab, with two bit commands. The first is a direction to move, and the second is a number for angle.

We opted for WASD commands. W is forward, A is left, S is backwards, D is right.
Number commands are as follows: 4 for 45 degree turn, 9 for 90 degree turn, 1 for go straight. 
EXAMPLES: W1 (go straight), L4 (turn left 45 degrees), R9 (turn right 90 degrees)
Connecting to the bluetooth module requires a linux machine, and a few steps. After correctly wiring the bluetooth including grounding the enable pin, a red led will blink quickly. This will continue until a connection is made, and then it will blink much slower. To connect, go to bluetooth and select HC-05. The default pin is ‘1234’. Once that portion of the connection is made open a linux terminal. In the linux terminal type the following commands in order:

sudo rfcomm bind rfcomm0 98:D3:41:FD:78:52
sudo screen /dev/rfcomm0 9600

The first command binds the linux machine to the hc-05 device. The second command shows the communication between machine and rover. The rover will ask for a two-bit command, and when a valid command is pushed will communicate the movement.


## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

Jordy A. Larrea Rodriguez  
[@CasuallyAlive](https://github.com/CasuallyAlive)

Cody Argyle
[@CodyArgyle](https://github.com/CodyArgyle)

Nicole Sundberg
[@nicolesundberg](https://github.com/nicolesundberg)

## Version History

* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
ex. * [awesome-readme](https://github.com/matiassingers/awesome-readme)
