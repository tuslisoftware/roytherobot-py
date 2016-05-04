# roytherobot-py

This Python module allows a user to control movement in the arm of [Roy the Robot](http://www.roemotion.com) made by Roemotion.

![](https://j.gifs.com/NkjlxL.gif)

### Usage

Example of basic usage:

```python
import roytherobot

# Let's assume the controller is connected to a USB port named '/dev/ttyUSB0' in Linux
# A context manager safely closes the serial device when finished
with roytherobot.Arm('/dev/ttyUSB0') as roy:

	# Enable all of the servos
	roy.enable_all_servos()
	
	# Open Roy's pinky finger to position 1100 over a 0.23 second duration
	roy.move(roy.PINKY, 1100, 0.23)
	
	# Close Roy's pinky finger to position 580 over a 0.33 second duration
	roy.move(roy.PINKY, 580, 0.33)
```

A full demonstration is also included in the project:

```python
import roytherobot.demo

# Let's assume the controller is connected to a USB port named 'COM3' in Windows
roytherobot.demo.run_demo('COM3')
```

### Documentation

There is no official documentation for roytherobot-py, however the python [source code](roytherobot/__init__.py) is small and well documented. The code is based off of the serial protocol documentation for the [Parallax Propeller Servo Controller](https://www.parallax.com/downloads/propeller-servo-controller-guide). Consult also the source code of the [demo program](roytherobot/demo/__init__.py) for a more detailed usage example.

### Dependencies

This pure-python module works in Python 3.x and legacy Python 2.x. It has been tested in Windows 7, OS X and Linux (Raspbian).

Requires [pySerial](https://github.com/pyserial/pyserial). Windows users will need to install [drivers for the Propeller](https://www.parallax.com/usbdrivers).

### Installation

Run the following in the terminal:

```
python setup.py install
```

### License

Released under the [MIT license](LICENSE).
