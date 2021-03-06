#!/usr/bin/env python

"""
Movement demonstration of Roy the Robot Arm

Author: Tusli Software LLC (info@tuslisoftware.com)
Written: May 1, 2016
Updated: May 1, 2016

This is a demo program using the Arm class. The main function is
`run_demo(port, debug)`. The motions of the arm are based on the demo VSA
software.

"""
from __future__ import (unicode_literals, division, absolute_import,
                        print_function)
import roytherobot


###############################################################################
# Convenience Functions
###############################################################################

def initialize_positions(roy):
    """
    Initial starting positions of each servo.
    Shows how to use roytherobot.Arm.set_position().

    Parameters
    ----------
    roy : roytherobot.Arm instance

    """
    roy.set_position(roy.WRIST_THUMB, 880)
    roy.set_position(roy.WRIST_PINKY, 630)
    roy.set_position(roy.PINKY, 580)
    roy.set_position(roy.RING, 525)
    roy.set_position(roy.MIDDLE, 515)
    roy.set_position(roy.INDEX, 1030)
    roy.set_position(roy.THUMB_TIP, 750)
    roy.set_position(roy.THUMB_BASE, 750)
    roy.set_position(roy.ARM_BASE, 795)


def open_and_close_fingers(roy):
    """
    Opens and closes each finger servo individually.
    Shows how to use roytherobot.Arm.move().

    Parameters
    ----------
    roy : roytherobot.Arm instance

    """
    # Set the servo positions
    roy.move(roy.PINKY, 1100, 0.23)
    roy.move(roy.PINKY, 580, 0.33)
    roy.move(roy.RING, 1154, 0.23)
    roy.move(roy.RING, 525, 0.33)
    roy.move(roy.MIDDLE, 1150, 0.23)
    roy.move(roy.MIDDLE, 515, 0.33)
    roy.move(roy.INDEX, 450, 0.23)
    roy.move(roy.INDEX, 1030, 0.33)
    roy.move(roy.THUMB_TIP, 1050, 0.23)
    roy.move(roy.THUMB_TIP, 750, 0.33)
    roy.move(roy.THUMB_BASE, 570, 0.23)
    roy.move(roy.THUMB_BASE, 750, 0.33)


def open_all_fingers_at_once(roy):
    """
    Opens all of the fingers at once.
    Shows how to use roytherobot.Arm.move() with multiple servos moving at
    once.

    Parameters
    ----------
    roy : roytherobot.Arm instance

    """
    channels = [roy.PINKY, roy.RING, roy.MIDDLE, roy.INDEX, roy.THUMB_TIP,
                roy.THUMB_BASE]
    pulse_widths = [1100, 1154, 1150, 450, 1050, 570]

    roy.move(channels, pulse_widths, 0.23)


def wrist_twist(roy):
    """
    Moves both wrist servos at the same time.

    Parameters
    ----------
    roy : roytherobot.Arm instance

    """
    channels = [roy.WRIST_THUMB, roy.WRIST_PINKY]
    pulse_widths = [[500, 1000], [1051, 456], [880, 640], [1000, 800],
                    [740, 516], [880, 640]]
    durations = [0.23, 0.49, 0.2, 0.2, 0.36, 0.2]

    # Go through each position and set the servos
    for idx in range(len(pulse_widths)):
        roy.move(channels, pulse_widths[idx], durations[idx])


def arm_twist(roy):
    """
    Twists the arm base left and right.

    Parameters
    ----------
    roy : roytherobot.Arm instance

    """
    channel = roy.ARM_BASE
    pulse_widths = [1200, 350, 788]
    durations = [0.46, 0.69, 0.47]

    for idx in range(len(pulse_widths)):
        roy.move(channel, pulse_widths[idx], durations[idx])


###############################################################################
# Main Function
###############################################################################

def run_demo(port, debug=False):
    """
    Runs demonstration using roytherobot module.

    Parameters
    ----------
    port : str
        The name of the serial port. Usually '/dev/ttyUSB0' for Linux,
        'COM3' for Windows, etc.
    debug : bool, optional
        Print debug messages.

    """
    # Initialize Roy the Robot
    with roytherobot.Arm(port, write_sleep=0.05, debug=debug) as roy:

        # Print some information about Roy
        print("FW Version: " + str(roy.controller.get_fw_version_number()))

        # Run the demo movements
        initialize_positions(roy)
        open_and_close_fingers(roy)
        open_all_fingers_at_once(roy)
        wrist_twist(roy)
        arm_twist(roy)

    print("Demo program finished.")
