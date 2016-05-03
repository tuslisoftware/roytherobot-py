# roytherobot-py
Python control module for Roemotion's Roy the Robot.

![](https://j.gifs.com/NkjlxL.gif)

This Python module allows a user to control a Roemotion Roy the Robot Arm. The serial protocol documentation for the Parallax Propeller Servo Controller can be found at <https://www.parallax.com/downloads/propeller-servo-controller-guide>. Information about Roy The Robot Arm can be found at <http://www.roemotion.com>.

This module works in Python 3.x and legacy Python 2.x. It has been tested in Windows 7, OS X and Linux (Raspbian).

### Installation
Run the following in the terminal:

```
python setup.py install
```
Windows users will need to install drivers for the Propeller. The drivers can be found at <https://www.parallax.com/usbdrivers>.
### Usage

Example of basic usage:

```python
import roytherobot

# Context manager safely closes the serial device at the end
with roytherobot.Arm('/dev/ttyUSB0') as roy:

	# Enable all of the servos
	roy.enable_all_servos()
	
	# Open Roy's pinky finger over a 0.23 second interval
	roy.move(roy.PINKY, 1100, 0.23)
	
	# Close Roy's pinky finger over a 0.33 second interval
	roy.move(roy.PINKY, 580, 0.33)
```

A full demonstration is also included in the project:

```python
import roytherobot.demo

roytherobot.demo.run_demo('COM3')
```
