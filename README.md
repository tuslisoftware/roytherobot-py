# roytherobot
Python control module for Roemotion's Roy the Robot.

This Python module allows a user to control a Roemotion Roy the Robot. Currently, only Roy the Robot Arm is supported, only those that use a Parallax Propeller Servo Controller USB (28830). The serial protocol documentation for the Propeller can be
found at <https://www.parallax.com/downloads/propeller-servo-controller-guide>. Information about Roy The Robot Arm can be found at <http://www.roemotion.com>.

This module works in Python 3.x and legacy Python 2.x. It has been tested in Windows 7, OS X and Linux (Raspbian).

To install, run the following in the terminal:
```
python setup.py install
```
Example of basic usage:

```python
import roytherobot

port = '/dev/ttyUSB0'
roy = roytherobot.Arm(port)
roy.enable_all_servos()
roy.move(roy.PINKY, 1100, 0.23)
roy.move(roy.PINKY, 580, 0.33)
roy.close()
```

Example using a "with" statement:

```python
import roytherobot

with roytherobot.Arm('/dev/ttyUSB0') as roy:
  roy.enable_all_servos()
  roy.move(roy.PINKY, 1100, 0.23)
  roy.move(roy.PINKY, 580, 0.33)
```

A full demonstration is also included in the project:

```python
import roytherobot.demo

roytherobot.demo.run_demo('/dev/ttyUSB0')
```
