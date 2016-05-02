#!/usr/bin/env python

"""
Control Module for Roy the Robot Arm

Author: Tusli Software LLC (info@tuslisoftware.com)
Written: May 1, 2016
Updated: May 1, 2016

This module allows a user to control a Roemotion Roy the Robot Arm. Visit
www.roemotion.com for more information about Roy the Robot Arm. The serial communication protocols
for the Parallax Propeller Servo Controller are published in Parallax Inc's Propeller Servo
Controller Guide: <https://www.parallax.com/downloads/propeller-servo-controller-guide>.

This module works in Python 3.x and legacy Python 2.x, and has been tested in OS X, Windows 7,
and Linux (Raspbian).

TODO:
    1. Add a convenience function to help the user find the port value for the Propeller.
"""

from __future__ import unicode_literals, division, absolute_import
import time
import struct
import math
from serial import Serial


def _iterable(obj):
    """ Return True of obj is iterable. """
    try:
        iter(obj)
    except TypeError:
        return False
    return True


class PropellerServoController28830(Serial):
    """
    Python class to control the Parallax Propeller Servo Controller USB (28830).

    NOTES:
    1. The 28830 board will be discontinued by Parallax in the near future.
    2. self.set_baud_rate(1) does not return the expected response.
    """

    MAX_PULSE_WIDTH = 2500
    MAX_RAMP_SPEED = 63
    CR = b'\r'

    def __init__(self, port, write_sleep=0.05):
        """
        Initialization. Parallax Propeller needs a baud rate of 2400, no parity, and one or
        two stop bits.

        Parameters
        ----------
        port : str
            The name of the serial port. Usually /dev/ttyUSB0 for Linux, COM3 for Windows, etc.
        write_sleep : float, default=0.05
            How long to wait after writing. For 2400 baud rate, less than 0.03 tends to cause
            communication errors.

        """
        Serial.__init__(self, port=port, baudrate=2400, parity='N', stopbits=1, timeout=5.0)
        self.write_sleep = write_sleep
        time.sleep(0.1)

    def _write(self, value):
        """
        Override serial.Serial.write() to sleep after writing.

        Parameters
        ----------
        value : bytearray
            Value to write.

        """

        self.write(value)
        time.sleep(self.write_sleep)

    def set_position(self, channel, ramp_speed, pulse_width):
        """
        Move the servo.
        Serial command: '!SC' + <channel> + <ramp_speed> + <pulse_width> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.
        ramp_speed : int
            Speed the servo moves to its new position, range 0-63.
        pulse_width : int
            Position, ranges are determined per servo.

        Raises
        ------
        PropellerError for invalid channel, ramp_speed or pulse_width values.

        """
        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if not isinstance(ramp_speed, int):
            raise PropellerError("Ramp speed must be an integer.")
        if not isinstance(pulse_width, int):
            raise PropellerError("Pulse width must be an integer.")
        if channel not in range(0, 16):
            raise PropellerError("Channel must be between 0 and 15.")
        if (ramp_speed < 0) or (ramp_speed > self.MAX_RAMP_SPEED):
            raise PropellerError("Ramp speed must be between 0 and " + str(self.MAX_RAMP_SPEED) + ".")
        if (pulse_width < 0) or (pulse_width > self.MAX_PULSE_WIDTH):
            raise PropellerError("Pulse width must be between 0 and " + str(self.MAX_PULSE_WIDTH) + ".")

        command = b"!SC" + struct.pack(b"<b", channel) + struct.pack(b"<b", ramp_speed) + struct.pack(b"<h", pulse_width) + self.CR
        self._write(command)

    def set_baud_rate(self, mode):
        """
        Sets the baud rate to either 2400 bps (mode 0) or 38.4 kbps (mode 1).
        Serial command: '!SCSBR' + <mode> + '\r'

        *WARNING* Mode 1 does NOT currently return 'BR' + <mode> for some reason,
        and is not confirmed to actually work.

        Parameters
        ----------
        mode : int, default=0
            Choose 0 (default) for 2400 bps and 1 for 38.4 kbps.

        Returns
        -------
        "BR" <mode>

        Raises
        ------
        PropellerError for invalid mode value.

        """
        baudrates = [2400, 38400]
        if mode not in range(0, 2):
            raise PropellerError("Mode must be 0 or 1.")

        if mode == 1:
            print("WARNING: Mode 1 is currently experimental, and has not been confirmed to work yet. No reply of 'BR' <mode> is received for mode 1.")

        command = b"!SCSBR" + struct.pack(b"<b", mode) + self.CR
        self._write(command)

        self.baudrate = baudrates[mode]

        # This is a momentary workaround. No reply is received for mode 1, which breaks the function.
        if mode == 0:
            retval = self.read(3)
            retcmd = retval[:2].decode("ascii")

            # Python 3.x
            if isinstance(retval[2], int):
                retmode = retval[2]

            # Python 2.x
            else:
                retmode = struct.unpack(b"<b", retval[2])[0]

            return retcmd + str(retmode)
        else:
            return None

    def set_software_port(self, mode):
        """
        Assigns the PSCU to act on commands sent to channels 0-15 (mode 0) or channels 16-31 (mode 1).
        Serial command: '!SCPSS' + <mode> + '\r'

        Parameters
        ----------
        mode : int, default=0
            Choose 0 (default) for channels 0-15 and 1 for channels 16-31.

        Returns
        -------
        "PM" <mode>

        Raises
        ------
        PropellerError for invalid mode value.
        
        """

        if mode not in range(0, 2):
            raise PropellerError("Mode must be 0 or 1.")

        command = b"!SCPSS" + struct.pack(b"<b", mode) + self.CR
        self._write(command)
        retval = self.read(3)
        retcmd = retval[:2].decode("ascii")

        # Python 3.x
        if isinstance(retval[2], int):
            retmode = retval[2]

        # Python 2.x
        else:
            retmode = struct.unpack(b"<b", retval[2])[0]

        return retcmd + str(retmode)

    def get_position(self, channel):
        """
        Returns the servo position for the channel.
        Serial command: '!SCRSP' + <channel> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.

        Returns
        -------
        Pulse width (in microseconds)

        Raises
        ------
        PropellerError for invalid channel value.
        
        """

        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if channel not in range(0, 16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCRSP" + struct.pack(b"<b", channel) + self.CR
        self._write(command)
        retval = self.read(3)

        # Convert the ASCII string to an integer
        return struct.unpack(b">h", retval[1:])[0]

    def enable_servo(self, channel):
        """
        Enables the servo.
        Serial command: '!SCPSE' + <channel> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.

        Raises
        ------
        PropellerError for invalid channel value.

        """

        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if channel not in range(0, 16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCPSE" + struct.pack(b"<b", channel) + self.CR
        self._write(command)

    def clear_eeprom(self):
        """
        Clears EEPROM values such as Port Mode, Servo Disabled, Startup Mode and Default Positions.
        Serial command: '!SCLEAR\r'

        Returns
        -------
        "CLR"

        """

        command = b"!SCLEAR" + self.CR
        self._write(command)
        retval = self.read(3)
        return retval.decode("ascii")

    def disable_servo(self, channel):
        """
        Disables a servo.
        Serial command: '!SCPSD' + <channel> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.

        Raises
        ------
        PropellerError for invalid channel value.
        """

        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if channel not in range(0, 16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCPSD" + struct.pack(b"<b", channel) + self.CR
        self._write(command)

    def set_startup_servo_mode(self, mode):
        """
        Sets whether the PSCU centers all servo channels on startup (mode 0) or 
        uses custom startup positions stored in EEPROM (mode 1).
        Serial command: '!SCEDD' + <mode> + '\r'

        Parameters
        ----------
        mode : int, default=0
            Center all servos on start up (mode 0, default) or use custom startup positions (mode 1).

        Returns
        -------
        "DL" <mode>

        Raises
        ------
        PropellerError for invalid mode value.
        """
        
        if mode not in range(0, 2):
            raise PropellerError("Mode must be 0 or 1.")

        command = b"!SCEDD" + struct.pack(b"<b", mode) + self.CR
        self._write(command)
        retval = self.read(3)
        retcmd = retval[:2].decode("ascii")
        # Python 3.x
        if isinstance(retval[2], int):
            retmode = retval[2]
        # Python 2.x
        else:
            retmode = struct.unpack(b"<b", retval[2])[0]
        return retcmd + str(retmode)

    def set_default_position(self, channel, pulse_width):
        """
        Set a new default position for individual servos instead of center.
        Serial command: '!SCD' + <channel> + <pulse_width> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.
        pulse_width : int
            Position, ranges depend on servo.

        Raises
        ------
        PropellerError for invalid channel and pulse_width values.

        """

        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if not isinstance(pulse_width, int):
            raise PropellerError("Pulse width must be an integer.")
        if channel not in range(0, 16):
            raise PropellerError("Channel must be between 0 and 15.")
        if (pulse_width < 0) or (pulse_width > self.MAX_PULSE_WIDTH):
            raise PropellerError("Pulse width must be between 0 and " + str(self.MAX_PULSE_WIDTH) + ".")

        command = b"!SCD" + struct.pack(b"<b", channel) + struct.pack(b"<h", pulse_width) + self.CR
        self._write(command)
        
    def get_fw_version_number(self):
        """
        Returns the firmware version number.
        Serial command: '!SCVER?\r'

        Returns
        -------
        Software version

        """

        command = b"!SCVER?" + self.CR
        self._write(command)
        return self.read(3).decode("ascii")


class RoyTheRobotError(Exception):
    """ Custom Roy the Robot Exception. """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class PropellerError(Exception):
    """ Custom Propeller Exception. """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Arm(object):
    """ Class to control the arm of Roy the Robot. """

    # Servo channels
    WRIST_THUMB = 0
    WRIST_PINKY = 1
    PINKY = 2
    RING = 3
    MIDDLE = 4
    INDEX = 5
    THUMB_TIP = 6
    THUMB_BASE = 7
    ARM_BASE = 8

    # Pulse width values (from experimentation and Brookshire software example)
    DEFAULT_PULSE_WIDTH = 750
    MIN_PULSE_WIDTH = [500, 456, 580, 525, 515, 450, 560, 570, 350]
    MAX_PULSE_WIDTH = [1051, 1000, 1100, 1154, 1150, 1030, 1050, 1050, 1200]

    PROPELLER_28830 = 0

    def __init__(self, port, write_sleep=0.05, servocontroller=0, debug=False):
        if servocontroller == self.PROPELLER_28830:
            self.servocontroller = PropellerServoController28830(port, write_sleep=write_sleep)
        else:
            raise RoyTheRobotError("Invalid servocontroller value.")
        self.write_sleep = write_sleep
        self.debug = debug
        if self.debug:
            print("Initialized roytherobot.Arm.")

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()
        return False

    def close(self):
        """ Close the servo controller. """
        if self.debug: print("Closing the servo controller.")
        self.servocontroller.close()

    def enable_all_servos(self):
        """ Enables all servos. """
        if self.debug: print("Enabling all servos.")
        for i in range(0, 9):
            self.enable_servo(i)

    def disable_all_servos(self):
        """ Disables all servos. """
        if self.debug: print("Disabling all servos.")
        for i in range(0, 9):
            self.disable_servo(i)

    def set_position(self, channel, pulse_width, ramp_speed=0):
        """
        Adds additional restrictions to parent class function and sets the ramp_speed default to 0.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.
        pulse_width : int
            Destination position, range depends on servo.
        ramp_speed : int, default=0
            Speed to move to the new position, range 0-63.

        Raises
        ------
        RoyTheRobotError for invalid channel or pulse_width values.

        """
        if self.debug: print("Setting channel " + str(channel) + " to " + str(pulse_width) + ".")
        # Make sure the user enters correct range values
        if channel not in range(0, 9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        if (pulse_width < self.MIN_PULSE_WIDTH[channel]):
            raise RoyTheRobotError("Pulse width for channel " + str(channel) + " must be greater than " + str(self.MIN_PULSE_WIDTH[channel]) + ". Pulse width entered is " + str(pulse_width) + ".")
        elif (pulse_width > self.MAX_PULSE_WIDTH[channel]):
            raise RoyTheRobotError("Pulse width for channel " + str(channel) + " must be less than " + str(self.MAX_PULSE_WIDTH[channel]) + ". Pulse width entered is " + str(pulse_width) + ".")
        
        self.servocontroller.set_position(channel, ramp_speed, pulse_width)

    def enable_servo(self, channel):
        """
        Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.

        Raises
        ------
        RoyTheRobotError for invalid channel value.

        """
        if self.debug: print("Channel " + str(channel) + " enabled.")
        if channel not in range(0, 9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        self.servocontroller.enable_servo(channel)

    def disable_servo(self, channel):
        """
        Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.

        Raises
        ------
        RoyTheRobotError for invalid channel value.

        """
        if self.debug: print("Channel " + str(channel) + " disabled.")
        if channel not in range(0, 9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        self.servocontroller.disable_servo(channel)

    def set_default_position(self, channel, pulse_width):
        """ Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.

        Raises
        ------
        RoyTheRobotError for invalid channel or pulse_width values.

        """
        if self.debug: print("Setting default position of channel " + str(channel) + " to " + str(pulse_width) + ".")
        if channel not in range(0, 9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        if (pulse_width < self.MIN_PULSE_WIDTH[channel]) or (pulse_width > self.MAX_PULSE_WIDTH[channel]):
            raise RoyTheRobotError("Pulse width for channel " + str(channel) + " must be between " + str(self.MIN_PULSE_WIDTH[channel]) + " and " + str(self.MAX_PULSE_WIDTH[channel]) + ".")
        self.servocontroller.set_default_position(channel, pulse_width)

    def get_position(self, channel):
        """
        Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.

        Raises
        ------
        RoyTheRobotError for invalid channel value.

        """
        if self.debug: print("Getting position of channel " + str(channel) + ".")
        if channel not in range(0, 9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        position = self.servocontroller.get_position(channel)
        return position

    def move(self, channels, pulse_widths, duration=0.3):
        """
        Moves multiple servos at the same time (sort of). It moves each servo in a staggered
        fashion.

        *WARNING*: duration does NOT include the time it takes to read the current pulse width values of
        each servo. The time it takes to read all of the servos is len(channels)*self.write_sleep.

        Parameters
        ----------
        channels : int or sequence of int
            Servo channel(s), range 0-8.
        pulse_widths : int or sequence of int
            Pulse width(s), range depends on servos.
        duration : float, optional
            How long to move all of the servos in seconds. Minimum is self.write_sleep.

        """
        if self.debug: print("Moving channel " + str(channels) + " to " + str(pulse_widths) + " in " + str(duration) + " seconds.")
        
        # Make sure the channels and pulse_widths are iterables
        if not _iterable(channels):
            channels = list([channels])

        if not _iterable(pulse_widths):
            pulse_widths = list([pulse_widths])

        # Make sure the lists are of equal length
        if len(channels) != len(pulse_widths):
            raise RoyTheRobotError("Number of channels and number of pulse widths must be equal.")

        # Make sure the duration is long enough
        if duration < self.write_sleep:
            raise RoyTheRobotError("Duration must be greater than or equal to self.write_sleep.")

        values = list(zip(channels, pulse_widths))

        # Number of iterations that can be used
        iterations = int(duration/self.write_sleep)

        # Avoid divide by zero
        if iterations != 0:

            # Start positions
            starts = dict()

            # Pulse width step sizes
            pw_steps = dict()

            for channel, pulse_width in values:

                # Start position of the servo
                pos = self.get_position(channel)
                starts[channel] = pos

                # Difference between start and end positions
                diff = pulse_width - pos

                # Get the pulse width step size for this servo
                pw_steps[channel] = int(diff/iterations)
            
            # Set the position of the servos
            for i in range(1, iterations):
                for channel, pulse_width in values:
                    self.set_position(channel, starts[channel] + (i*pw_steps[channel]))

        # Set the final servo positions
        for channel, pulse_width in values:
            self.set_position(channel, pulse_width)
            