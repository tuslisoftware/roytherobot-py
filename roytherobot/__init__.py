#!/usr/bin/env python

"""
Control Module for Roy the Robot Arm

Author: Tusli Software LLC (info@tuslisoftware.com)
Written: May 1, 2016
Updated: May 1, 2016

This module allows a user to control the arm of Roy the Robot, built by
Roemotion. Visit <www.roemotion.com> for more information about Roy the Robot.
The serial communication protocols for the Parallax Propeller Servo Controller
are published in Parallax Inc's Propeller Servo Controller Guide:
<https://www.parallax.com/downloads/propeller-servo-controller-guide>.

This pure-python module works in Python 3.x and legacy Python 2.x, and has been
tested in OS X, Windows 7, and Linux (Raspbian). It is released under the MIT
license.

The servo position is set by sending it a pulse width modulation (PWM) signal.
The signal is a repeated pulse with a variable width in microseconds. The
pulse width range depends on the individual servo, but most of the arm servos
(with the exception of the arm base) range from about 500 to 1000 microseconds.
The direction of movement depends on the orientation of the servo; for example,
sending the pinky finger a 1000us pulse width will open the pinky finger, while
sending the same value to the index finger will close it.

TODO:
    1. Add a convenience function to help the user find the USB port that is
    connected to the microcontroller.

"""
from __future__ import (unicode_literals, division, absolute_import,
                        print_function)
import time
import struct
import math
from serial import Serial


def _iterable(obj):
    """ Returns True if `obj` is iterable. """
    try:
        iter(obj)
    except TypeError:
        return False
    return True


class Propeller28830(Serial):
    """
    Python class to control the Parallax Propeller Servo Controller USB
    (28830).

    Notes
    -----
    1. The 28830 board will be discontinued by Parallax in the near future.
    2. self.set_baud_rate(1) does not return the expected response.

    """
    MAX_PULSE_WIDTH = 2500
    MAX_RAMP_SPEED = 63
    CR = b'\r'

    def __init__(self, port, write_sleep=0.05):
        """
        Initialization. Parallax Propeller needs a baud rate of 2400, no
        parity, and one or two stop bits.

        Parameters
        ----------
        port : str
            The name of the serial port. Usually '/dev/ttyUSB0' for Linux,
            'COM3' for Windows, etc.
        write_sleep : float, optional
            How long to wait in seconds after writing. For 2400 baud rate,
            less than 0.03 tends to cause communication errors.

        """
        Serial.__init__(self, port=port, baudrate=2400, parity='N',
                        stopbits=1, timeout=5.0)
        self.write_sleep = write_sleep
        time.sleep(0.1)

    def _write(self, value):
        """
        Overrides serial.Serial.write() to sleep after writing.

        Parameters
        ----------
        value : bytearray
            Value to write.

        """
        self.write(value)
        time.sleep(self.write_sleep)

    def set_position(self, channel, ramp_speed, pulse_width):
        """
        Moves the servo.
        Serial command: '!SC' + <channel> + <ramp_speed> + <pulse_width> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.
        ramp_speed : int
            Speed the servo moves to its new position, range 0-63.
        pulse_width : int
            Pulse width in microseconds determines the position of the servo.
            The value ranges depend on the individual servo.

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
        if channel not in range(16):
            raise PropellerError("Channel must be between 0 and 15.")

        max_pw = self.MAX_PULSE_WIDTH
        max_rs = self.MAX_RAMP_SPEED

        if (ramp_speed < 0) or (ramp_speed > max_rs):
            msg = "Ramp speed must be between 0 and %d." % max_rs
            raise PropellerError(msg)
        if (pulse_width < 0) or (pulse_width > max_pw):
            msg = "Pulse width must be between 0 and %d." % max_pw
            raise PropellerError(msg)

        ch = struct.pack(b"<b", channel)
        rs = struct.pack(b"<b", ramp_speed)
        pw = struct.pack(b"<h", pulse_width)

        command = b"!SC" + ch + rs + pw + self.CR
        self._write(command)

    def set_baud_rate(self, mode=0):
        """
        Sets the baud rate to either 2400 bps (mode 0) or 38.4 kbps (mode 1).
        Serial command: '!SCSBR' + <mode> + '\r'

        *WARNING* Mode 1 does NOT currently return 'BR' + <mode> for some
        reason, and is not confirmed to actually work.

        Parameters
        ----------
        mode : int, optional
            Choose 0 (default) for 2400 bps and 1 for 38.4 kbps.

        Returns
        -------
        out : str or None
            String equal to: "BR" + <mode>.

        Raises
        ------
        PropellerError for invalid mode value.

        """
        baudrates = [2400, 38400]
        if mode not in range(2):
            raise PropellerError("Mode must be 0 or 1.")

        if mode == 1:
            warning = ("WARNING: Mode 1 is currently experimental, and has "
                       "not been confirmed to work yet. No reply of 'BR' + "
                       "<mode> is received for mode 1."
                       )
            print(warning)

        command = b"!SCSBR" + struct.pack(b"<b", mode) + self.CR
        self._write(command)

        self.baudrate = baudrates[mode]

        # This is a momentary workaround
        # No reply is received for mode 1, which breaks the function
        out = None
        if mode == 0:
            retval = self.read(3)
            retcmd = retval[:2].decode("ascii")

            # Python 3.x
            if isinstance(retval[2], int):
                retmode = retval[2]

            # Python 2.x
            else:
                retmode = struct.unpack(b"<b", retval[2])[0]

            out = retcmd + str(retmode)
        return out

    def set_software_port(self, mode=0):
        """
        Assigns the PSCU to act on commands sent to channels 0-15 (mode 0) or
        channels 16-31 (mode 1).
        Serial command: '!SCPSS' + <mode> + '\r'

        Parameters
        ----------
        mode : int, optional
            Choose 0 (default) for channels 0-15 and 1 for channels 16-31.

        Returns
        -------
        out : str
            String equal to: "PM" + <mode>.

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

        out = retcmd + str(retmode)
        return out

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
        pulse_width : int
            Pulse width in microseconds determines the position of the servo.
            The value ranges depend on the individual servo.

        Raises
        ------
        PropellerError for invalid channel value.

        """
        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if channel not in range(16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCRSP" + struct.pack(b"<b", channel) + self.CR
        self._write(command)
        retval = self.read(3)

        # Convert the ASCII string to an integer
        pulse_width = struct.unpack(b">h", retval[1:])[0]
        return pulse_width

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
        if channel not in range(16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCPSE" + struct.pack(b"<b", channel) + self.CR
        self._write(command)

    def clear_eeprom(self):
        """
        Clears EEPROM values such as Port Mode, Servo Disabled, Startup Mode
        and Default Positions.
        Serial command: '!SCLEAR\r'

        Returns
        -------
        out : str
            String equal to: "CLR".

        """
        command = b"!SCLEAR" + self.CR
        self._write(command)
        retval = self.read(3)
        out = retval.decode("ascii")
        return out

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
        if channel not in range(16):
            raise PropellerError("Channel must be between 0 and 15.")

        command = b"!SCPSD" + struct.pack(b"<b", channel) + self.CR
        self._write(command)

    def set_startup_servo_mode(self, mode=0):
        """
        Sets whether the PSCU centers all servo channels on startup (mode 0)
        or uses custom startup positions stored in EEPROM (mode 1).
        Serial command: '!SCEDD' + <mode> + '\r'

        Parameters
        ----------
        mode : int, optional
            Center all servos on start up (mode 0, default) or use custom
            startup positions (mode 1).

        Returns
        -------
        out : str
            String equal to: "DL" + <mode>.

        Raises
        ------
        PropellerError for invalid mode value.

        """
        if mode not in range(2):
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

        out = retcmd + str(retmode)
        return out

    def set_default_position(self, channel, pulse_width):
        """
        Set a new default position for individual servos instead of center.
        Serial command: '!SCD' + <channel> + <pulse_width> + '\r'

        Parameters
        ----------
        channel : int
            Servo channel, range 0-15.
        pulse_width : int
            Pulse width in microseconds determines the position of the servo.
            The value ranges depend on the individual servo.

        Raises
        ------
        PropellerError for invalid channel and pulse_width values.

        """
        if not isinstance(channel, int):
            raise PropellerError("Channel must be an integer.")
        if not isinstance(pulse_width, int):
            raise PropellerError("Pulse width must be an integer.")
        if channel not in range(16):
            raise PropellerError("Channel must be between 0 and 15.")

        max_pw = self.MAX_PULSE_WIDTH

        if (pulse_width < 0) or (pulse_width > max_pw):
            msg = "Pulse width must be between 0 and %d." % max_pw
            raise PropellerError(msg)

        ch = struct.pack(b"<b", channel)
        pw = struct.pack(b"<h", pulse_width)

        command = b"!SCD" + ch + pw + self.CR
        self._write(command)

    def get_fw_version_number(self):
        """
        Returns the firmware version number.
        Serial command: '!SCVER?\r'

        Returns
        -------
        version : str
            Firmware version number string.

        """
        command = b"!SCVER?" + self.CR
        self._write(command)
        version = self.read(3).decode("ascii")
        return version


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

    def __init__(self, port, write_sleep=0.05, controller='PROPELLER_28830',
                 debug=False):
        """
        Initialization.

        Parameters
        ----------
        port : str
            The name of the serial port. Usually '/dev/ttyUSB0' for Linux,
            'COM3' for Windows, etc.
        write_sleep : float, optional
            How long to wait in seconds after writing. For 2400 baud rate,
            less than 0.03 tends to cause communication errors.
        controller : str, optional
            Only one controller board is supported currently,
            'PROPELLER_28830', but this board has been deprecated by the
            manufacturer and will likely be replaced in the future.
        debug : bool, optional
            Print debug messages.

        Raises
        ------
        RoyTheRobotError if controller is not implemented.

        """
        if controller == 'PROPELLER_28830':
            self.controller = Propeller28830(port, write_sleep=write_sleep)
        else:
            raise RoyTheRobotError("Invalid controller board.")
        self.write_sleep = write_sleep
        self.debug = debug
        self.enable_all_servos()
        self._debug("Initialized roytherobot.Arm.")

    def __enter__(self):
        """ Enters a context manager. """
        return self

    def __exit__(self, error_type, value, traceback):
        """ Closes the serial connection when exiting a context manager. """
        self.close()
        return False

    def _debug(self, message):
        """ Prints message if the debug flag is True. """
        if self.debug:
            print(message)

    def close(self):
        """ Close the servo controller. """
        self._debug("Closing the servo controller.")
        self.controller.close()

    def enable_all_servos(self):
        """ Enables all servos. """
        self._debug("Enabling all servos.")
        for i in range(9):
            self.enable_servo(i)

    def disable_all_servos(self):
        """ Disables all servos. """
        self._debug("Disabling all servos.")
        for i in range(9):
            self.disable_servo(i)

    def set_position(self, channel, pulse_width, ramp_speed=0):
        """
        Jump to a new position in one step.

        *WARNING*: the servo may refuse to move at all if the requested
        position is too far from its current position. Try splitting the motion
        into smaller steps instead of one large jump. The
        `move(channels, pulse_widths, duration)` method tries to do this
        unless `duration` is too short. In general, the `move` method is
        preferred over `set_position` for most use cases. Think of `move` as
        the high-level function and `set_position` as the low-level function
        that can both accomplish the same goal.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.
        pulse_width : int
            Destination position, range depends on servo.
        ramp_speed : int, optional
            Speed to move to the new position, range 0-63. Strangely, this
            seems to have no observable effect, so far.

        Raises
        ------
        RoyTheRobotError for invalid channel or pulse_width values.

        """
        self._debug("Setting channel %d to %d." % (channel, pulse_width))
        # Make sure the user enters correct range values
        if channel not in range(9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        if pulse_width < self.MIN_PULSE_WIDTH[channel]:
            min_pw = self.MIN_PULSE_WIDTH[channel]
            msg = ("Pulse width for channel %d "
                   "must be greater than %d.") % (channel, min_pw)
            raise RoyTheRobotError(msg)
        elif pulse_width > self.MAX_PULSE_WIDTH[channel]:
            max_pw = self.MAX_PULSE_WIDTH[channel]
            msg = ("Pulse width for channel %d "
                   "must be less than %d.") % (channel, max_pw)
            raise RoyTheRobotError(msg)

        self.controller.set_position(channel, ramp_speed, pulse_width)

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
        self._debug("Channel " + str(channel) + " enabled.")
        if channel not in range(9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        self.controller.enable_servo(channel)

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
        self._debug("Channel " + str(channel) + " disabled.")
        if channel not in range(9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        self.controller.disable_servo(channel)

    def set_default_position(self, channel, pulse_width):
        """ Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.
        pulse_width : int
            Pulse width in microseconds determines the position of the servo.
            The value ranges depend on the individual servo.

        Raises
        ------
        RoyTheRobotError for invalid channel or pulse_width values.

        """
        self._debug(("Setting default position "
                     "of channel %d to %d.") % (channel, pulse_width))
        if channel not in range(9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")

        # Min and max pulse widths
        min_pw = self.MIN_PULSE_WIDTH[channel]
        max_pw = self.MAX_PULSE_WIDTH[channel]
        if (pulse_width < min_pw) or (pulse_width > max_pw):
            msg = ("Pulse width for channel %d must be between "
                   "%d and %d.") % (channel, min_pw, max_pw)
            raise RoyTheRobotError(msg)
        self.controller.set_default_position(channel, pulse_width)

    def get_position(self, channel):
        """
        Adds additional restrictions to parent class function.

        Parameters
        ----------
        channel : int
            Servo channel, range 0-8.

        Returns
        -------
        pulse_width : int
            Pulse width in microseconds determines the position of the servo.
            The value ranges depend on the individual servo.

        Raises
        ------
        RoyTheRobotError for invalid channel value.

        """
        if channel not in range(9):
            raise RoyTheRobotError("Channel must be between 0 and 8.")
        pulse_width = self.controller.get_position(channel)
        self._debug("Position of channel %d: %d." % (channel, pulse_width))
        return pulse_width

    def move(self, channels, pulse_widths, duration=0.3):
        """
        Moves multiple servos at the same time (sort of). It moves each servo
        in a staggered fashion.

        *WARNING*: the servo may refuse to move at all if duration is too
        short.

        *NOTE*: duration does NOT include the time it takes to read the
        current pulse-width values of each servo. The time it takes to read
        all of the servos is len(channels)*self.write_sleep.

        Parameters
        ----------
        channels : int or sequence of int
            Servo channel(s), range 0-8.
        pulse_widths : int or sequence of int
            Pulse width(s), range depends on servos.
        duration : float, optional
            How long to move all of the servos in seconds. Minimum is
            self.write_sleep.

        Raises
        ------
        RoyTheRobotError for invalid channels, pulse_widths or duration values.

        """
        self._debug("Moving channel " + str(channels) + " to " + str(pulse_widths) +
                    " in %g seconds." % duration)

        # Make sure the channels and pulse_widths are iterables
        if not _iterable(channels):
            channels = list([channels])

        if not _iterable(pulse_widths):
            pulse_widths = list([pulse_widths])

        # Make sure the lists are of equal length
        if len(channels) != len(pulse_widths):
            msg = "Number of channels and pulse widths must be equal."
            raise RoyTheRobotError(msg)

        # Make sure the duration is long enough
        if duration < self.write_sleep:
            msg = "Duration must be greater than or equal to self.write_sleep."
            raise RoyTheRobotError(msg)

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
                    start = starts[channel]
                    self.set_position(channel, start + (i*pw_steps[channel]))

        # Set the final servo positions
        for channel, pulse_width in values:
            self.set_position(channel, pulse_width)
