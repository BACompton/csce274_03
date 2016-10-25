#!/usr/bin/python

import struct
import math

import serial_inf

# =============================================================================
#                   Constants for iRobot Create 2
# =============================================================================

SENSOR_UPDATE_WAIT = .015   # Time in between sensor updates
WHEEL_BASE = 235.0

_BAUD_RATE = 115200      # iRobot Create 2 Default baud rate
_TIMEOUT = 1             # The default read timeout to avoid indefinite blocking
_SENSORS_OPCODE = "142"  # Opcode for the sensors command
_BYTE = 0xFF             # Helps isolate a byte
_BYTE_SIZE = 8           # The number of bit in a byte of iRobot Create 2
_WHEEL_DIAMETER = 72.0
_COUNTS_PER_REV = 508.8


class State:
    """ Represents the different states of the iRobot Create 2. The value at
        each state is the necessary command to enter that state.

        There is no command to transition back to the START state, so the
        transition from the START state is one way.
    """
    START = ""
    RESET = "7"
    STOP = "173"
    PASSIVE = "128"
    SAFE = "131"
    FULL = "132"


class Button:
    """ Represents the different buttons on the iRobot Create 2. The value of
        each button are the corresponding bit it refers to in the button packet.

        This also contains the packet id, and the number of data bytes to read.
    """
    # Buttons
    CLOCK = 0x80
    SCHEDULE = 0x40
    DAY = 0x20
    HOUR = 0x10
    MINUTE = 0x08
    DOCK = 0x04
    SPOT = 0x02
    CLEAN = 0x01

    # Packet Information
    PACKET_ID = 18
    DATA_BYTES = 1


class Drive:
    """ Represents special cases for the radius of the iRobot Create 2's
        drive command, and the bounds for velocity and radius.
    """
    # Special Cases for the radius
    STRAIGHT = 0x8000
    STRAIGHT_ALT = 0x7FFF
    TURN_CW = 0xFFFF
    TURN_CCW = 0x0001

    # Bounds established by iRobot Create 2's OI specifications.
    MAX_VEL = 500
    MIN_VEL = -500
    MAX_RAD = 2000
    MIN_RAD = -2000
    MAX_ENCODER = 32767
    MIN_ENCODER = -32768
    MAX_DIST = MAX_ENCODER*math.pi*_WHEEL_DIAMETER/_COUNTS_PER_REV
    MIN_DIST = MIN_ENCODER*math.pi*_WHEEL_DIAMETER/_COUNTS_PER_REV

    # Packet Information
    ENCODER_R = 43
    ENCODER_L = 44
    ENCODER_BYTES = 2

    @staticmethod
    def list():
        """ Creates a iterable list of all encoders.
        :return:
            A list of all encoders.
        """
        return [Drive.ENCODER_L,
                Drive.ENCODER_R]


class Bump:
    """ Represents the two different bumps on the IRobot Create 2. The value
        of each bump are the corresponding bit it refers to in the bump and
        wheel drop packet.

        This also contains the packet id, and the number of data bytes to read.
    """

    # Bumps
    BUMP_L = 0x02
    BUMP_R = 0x01

    # Packet Information
    PACKET_ID = 7
    DATA_BYTES = 1


class WheelDrop:
    """ Represents the two different wheel drops on the IRobot Create 2. The
        value of each wheel drop are the corresponding bit it refers to in the
        bump and wheel drop packet.

        This also contains the packet id, and the number of data bytes to read.
    """

    # Wheel Drops
    WHEEL_DROP_L = 0x08
    WHEEL_DROP_R = 0x04

    # Packet Information
    PACKET_ID = 7
    DATA_BYTES = 1


class Cliff:
    """ Represents the cliff and virtual wall sensors on the IRobot Create 2.
        The value of each sensor are the corresponding packet ID.

        This also contains the number of data bytes to read.
    """

    CLIFF_L = 9
    CLIFF_FL = 10
    CLIFF_R = 11
    CLIFF_FR = 12
    VIRTUAL_WALL = 13
    DATA_BYTES = 1

    @staticmethod
    def list():
        """ Creates a iterable list of all cliff and virtual wall sensors.
        :return:
            A list of all cliff and virtual wall sensors.
        """
        return [Cliff.CLIFF_L,
                Cliff.CLIFF_FL,
                Cliff.CLIFF_R,
                Cliff.CLIFF_FR,
                Cliff.VIRTUAL_WALL]

# =============================================================================
#                       iRobot Create 2's Interface
# =============================================================================


class Robot:
    """ Represents a interface for iRobot Create 2 over a serial connection.

        Attributes:
            state: A State value used to indicate the robot's current state.
    """
    state = None

    _serial_conn = None
    _warning_song_num = None

    def __init__(self, port, buad=_BAUD_RATE, timeout=_TIMEOUT, start=True):
        """ Initializes a robot by first establishing the serial connection to
            the robot. and then sending the start command the robot. This
            command will set the robot's mode to PASSIVE, and grant the
            ability to send other command.

        :param port:
            The serial port to the robot
        :param buad:
            The buad rate of the robot
        :param timeout:
            The read timeout for the robot
        :param start:
            Flags the robot to enter the PASSIVE state. If this is false,
            the robot's state will need to be changed from START to
            PASSIVE before any other commands can be sent.
        :return:
        """
        self._serial_conn = serial_inf.SerialConn(port, buad, timeout)
        self.state = State.START

        if start:
            self.change_state(State.PASSIVE)

    # -------------------------------------------------------------------- #
    # -                    Command Issuing Methods                       - #
    # -------------------------------------------------------------------- #

    def change_state(self, new_state):
        """ Changes the robot into the provided state provided it is a different
            state and the new state is not State.START.

        :type new_state State:
        :param new_state:
            The state that the robot should transition into.
        :return:
            True if the change was successful, otherwise false.
        """
        if new_state != self.state and new_state != State.START:
            self._serial_conn.send_command(new_state)
            self.state = new_state
            return True
        return False

    def drive(self, velocity, radius):
        """ Issues the drive command for the robot. This method specifically
            calls the drive command with opcode 137 which is only available
            to a robot is the SAFE or FULL state.

            Both arguments will be interrupt as 16 bit values in two's
            complement representation. This means this method supports
            explicit integer values(i.e. -1, 500, -150) and 16 bit
            integers(i.e 0xFFFF. 0x7FFF).

            All the special cases for this command can be found in the Drive
            class.
        :param velocity:
            The velocity of the robot in millimeters per second. This can
            range from -500 to 500 mm/s.
        :param radius:
            The radius the robot should turn in mm. This can range from
            -2000 to 2000 mm.
        """

        # Bounds the velocity and radius to the robot's specified range
        bound_vel = self._convert_bound(velocity, Drive.MIN_VEL, Drive.MAX_VEL)
        # Captures the special cases that fall outside the bounds for radius
        if radius == Drive.STRAIGHT or radius == Drive.STRAIGHT_ALT:
            bound_rad = radius
        else:
            bound_rad = self._convert_bound(radius,
                                            Drive.MIN_RAD,
                                            Drive.MAX_RAD)

        # Separates and prepares the data for encoding
        data = (bound_vel >> _BYTE_SIZE & _BYTE,
                bound_vel & _BYTE,
                bound_rad >> _BYTE_SIZE & _BYTE,
                bound_rad & _BYTE)

        # Sends drive command to robot
        self._serial_conn.send_command("137 %s %s %s %s" % data)

    def drive_direct(self, vel_r=0, vel_l=0):
        """ Controls the motors of the robot explicitly in velocity.

            Velocity range is -500 to 500 mm/s.

        :param vel_r:
            The velocity of the right wheel. This is a 16-bit number.
        :param vel_l:
            The velocity of the left wheel. This is a 16 bit number.
        """
        bound_vel_r = self._convert_bound(vel_r, Drive.MIN_VEL, Drive.MAX_VEL)
        bound_vel_l = self._convert_bound(vel_l, Drive.MIN_VEL, Drive.MAX_VEL)

        data = (bound_vel_r >> _BYTE_SIZE & _BYTE,
                bound_vel_r & _BYTE,
                bound_vel_l >> _BYTE_SIZE & _BYTE,
                bound_vel_l & _BYTE)

        self._serial_conn.send_command("145 %s %s %s %s" % data)

    def set_warning_song(self, song_number):
        """ Sets the warning song to the specified song number. This should
            be called before playing the warning song.

        :param song_number:
            The song number. Range: 0-4
        :return:
        """
        self._warning_song_num = int(math.fabs(song_number)) % 5

        # Song is in c major scale and is the 5th (G) to the 3rd (E).
        cmd = "140 " + str(self._warning_song_num) + " 2 67 16 64 16"

        self._serial_conn.send_command(cmd)

    def play_warning_song(self):
        """ Plays the warning song
        """
        if self._warning_song_num is None:
            self.set_warning_song(0)

        self._serial_conn.send_command("141 " + str(self._warning_song_num))

    # -------------------------------------------------------------------- #
    # -                     Sensor Reading Methods                       - #
    # -------------------------------------------------------------------- #

    def read_button(self, button):
        """ Reads the provided button's bit from the Buttons packet. This method
            is available to a robot in the PASSIVE, SAFE, or FULL state.

        :param button:
            The button to read
        :return:
            The boolean value of the corresponding button.
        """
        data = self._read_packet(Button.PACKET_ID, Button.DATA_BYTES)

        # Gets first byte
        if len(data) == Button.DATA_BYTES:
            byte = struct.unpack("B", data)[0]
            return bool(byte & button)
        else:
            return False

    def read_buttons(self):
        """ Reads all the buttons on the iRobot Create 2. This method
            is available to a robot in the PASSIVE, SAFE, or FULL state.

        :return:
            A dictionary of all the buttons' values. Each record is
            addressed by the value in Button. Thus, the individual button
            values can be acquired by like so:
                clean = someRobot.read_buttons()[Button.CLEAN]
        """
        data = self._read_packet(Button.PACKET_ID, Button.DATA_BYTES)

        # Gets first byte
        if len(data) == Button.DATA_BYTES:
            byte = struct.unpack("B", data)[0]
            return {
                Button.CLEAN: bool(byte & Button.CLEAN),
                Button.SPOT: bool(byte & Button.SPOT),
                Button.DOCK: bool(byte & Button.DOCK),
                Button.MINUTE: bool(byte & Button.MINUTE),
                Button.HOUR: bool(byte & Button.HOUR),
                Button.DAY: bool(byte & Button.DAY),
                Button.SCHEDULE: bool(byte & Button.SCHEDULE),
                Button.CLOCK: bool(byte & Button.CLOCK)
            }
        else:
            return {
                Button.CLEAN: False,
                Button.SPOT: False,
                Button.DOCK: False,
                Button.MINUTE: False,
                Button.HOUR: False,
                Button.DAY: False,
                Button.SCHEDULE: False,
                Button.CLOCK: False
            }

    def read_bump(self, bump):
        """ Reads the provided bump's bit from the Bump and wheel drop packet.
            This method is available to a robot in the PASSIVE, SAFE,
            or FULL state.

        :param bump:
            The bump to read
        :return:
            The boolean value of the corresponding bump.
        """
        data = self._read_packet(Bump.PACKET_ID, Bump.DATA_BYTES)

        if len(data) == Bump.DATA_BYTES:
            byte = struct.unpack("B", data)[0]
            return bool(byte & bump)
        else:
            return False

    def read_bumps(self):
        """ Reads all the bumps on the iRobot Create 2. This method
            is available to a robot in the PASSIVE, SAFE, or FULL state.

        :return:
            A dictionary of all the bumps' values. Each record is
            addressed by the value in Bump. Thus, the individual bump
            values can be acquired by like so:
                bump_l = someRobot.read_bumps()[Bump.BUMP_L]
        """
        data = self._read_packet(Bump.PACKET_ID, Bump.DATA_BYTES)

        if len(data) == Bump.DATA_BYTES:
            byte = struct.unpack("B", data)[0]

            return {
                Bump.BUMP_L: bool(byte & Bump.BUMP_L),
                Bump.BUMP_R: bool(byte & Bump.BUMP_R)
            }
        else:
            return {
                Bump.BUMP_L: False,
                Bump.BUMP_R: False
            }

    def read_wheel_drop(self, wheel_drop):
        """ Reads the provided wheel drop's bit from the Bump and wheel
            drop packet. This method is available to a robot in the PASSIVE,
            SAFE, or FULL state.

        :param wheel_drop:
            The wheel drop to read
        :return:
            The boolean value of the corresponding wheel drop.
        """
        data = self._read_packet(WheelDrop.PACKET_ID, WheelDrop.DATA_BYTES)

        if len(data) == WheelDrop.DATA_BYTES:
            byte = struct.unpack("B", data)[0]
            return bool(byte & wheel_drop)
        else:
            return False

    def read_wheel_drops(self):
        """ Reads all the wheel drops on the iRobot Create 2. This method
            is available to a robot in the PASSIVE, SAFE, or FULL state.

        :return:
            A dictionary of all the wheel drops' values. Each record is
            addressed by the value in WheelDrop. Thus, the individual wheel
            drop values can be acquired by like so:
               drop_l = someRobot.read_wheel_drops()[WheelDrop.WHEEL_DROP_L]
        """
        data = self._read_packet(WheelDrop.PACKET_ID, WheelDrop.DATA_BYTES)

        if len(data) == WheelDrop.DATA_BYTES:
            byte = struct.unpack("B", data)[0]

            return {
                WheelDrop.WHEEL_DROP_L: bool(byte & WheelDrop.WHEEL_DROP_L),
                WheelDrop.WHEEL_DROP_R: bool(byte & WheelDrop.WHEEL_DROP_R)
            }
        else:
            return {
                WheelDrop.WHEEL_DROP_L: False,
                WheelDrop.WHEEL_DROP_R: False
            }

    def read_bump_wheel_drop(self):
        """ Reads all the wheel drops and bumps on the iRobot Create 2 in the
            bump and wheel drop packet. This method is available to a robot
            in the PASSIVE, SAFE, or FULL state.

        :return:
            A dictionary of all the bump and wheel drop values. Each record
            is addressed by the value in Bump or WheelDrop. Thus, the
            individual bump or wheel drop values can be acquired by like so:
                bump_l = someRobot.read_bump_wheel_drop()[Bump.BUMP_L]
        """

        # Bump and Wheel drop packet information is interchangeable
        # in this case.
        data = self._read_packet(WheelDrop.PACKET_ID, WheelDrop.DATA_BYTES)

        if len(data) == WheelDrop.DATA_BYTES:
            byte = struct.unpack("B", data)[0]

            return {
                Bump.BUMP_L: bool(byte & Bump.BUMP_L),
                Bump.BUMP_R: bool(byte & Bump.BUMP_L),
                WheelDrop.WHEEL_DROP_L: bool(byte & WheelDrop.WHEEL_DROP_L),
                WheelDrop.WHEEL_DROP_R: bool(byte & WheelDrop.WHEEL_DROP_R)
            }
        else:
            return {
                Bump.BUMP_L: False,
                Bump.BUMP_R: False,
                WheelDrop.WHEEL_DROP_L: False,
                WheelDrop.WHEEL_DROP_R: False
            }

    def read_cliff(self, cliff):
        """ Reads the provided cliff or virtual wall sensor. This method is
            available to a robot in the PASSIVE, SAFE, or FULL state.

        :param cliff:
            The cliff sensor to read
        :return:
            The value of the specified cliff sensor.
        """
        data = self._read_packet(cliff, Cliff.DATA_BYTES)

        if len(data) == Cliff.DATA_BYTES:
            byte = struct.unpack("B", data)[0]

            return bool(byte)
        else:
            return False

    def read_cliffs(self):
        """ Reads all the cliff and virtual wall sensors. This method is
            available to a robot in the PASSIVE, SAFE, or FULL state.

        :return:
            A dictionary of all the cliff and virtual wall sensors. Each
            record is addressed by the value in Cliff. Thus, the
            individual cliff values can be acquired by like so:
                cliff_l = someRobot.read_cliff()[Cliff.CLIFF_L]
        """
        cliff_list = Cliff.list()
        rtn = {}

        for clf in cliff_list:
            rtn[clf] = self.read_cliff(clf)

        return rtn

    def read_encoder(self, encoder):
        """ Reads the specified encoder's count.

        :type encoder Drive:
        :param encoder:
            The encoder to read.
        :return:
            The distance of represented by the encoder's count.
        """
        counts = self._read_encoder_raw(encoder)

        return counts*math.pi*_WHEEL_DIAMETER / _COUNTS_PER_REV

    def read_encoders(self):
        """ Reads the count of the encoders.

        :return:
            A dictionary of each encoder's count as distance. Can be
            referenced by using the encoder values in Drive.
        """
        enc_list = Drive.list()
        rtn = {}

        for enc in enc_list:
            rtn[enc] = self.read_encoder(enc)

        return rtn

    # -------------------------------------------------------------------- #
    # -                         Helper Methods                           - #
    # -------------------------------------------------------------------- #

    def distance(self, ref_dist, new_dist=None, forward=True):
        """ Calculates the distance between two encoder counts represented
            as distances in mm.

        :param ref_dist:
            The reference encoder distance
        :param new_dist:
            The new encoder distance
        :param forward:
            Flag used to determine turnover
        :return:
            The averaged distance of traveled by each encoder.
        """
        if new_dist is None:
            new_dist = self.read_encoders()

        enc_count = 0
        enc_sum = 0
        # For each encoder in both dictionaries increment encoder count and
        # add the difference between the encoder's value to a running summation.
        for dist in ref_dist:
            if dist in new_dist:
                enc_count += 1
                enc_sum += Robot._encoder_diff(ref_dist[dist],
                                               new_dist[dist],
                                               forward)

        # Average the difference between encoders values
        return enc_sum / enc_count

    def angle(self, ref_angle, new_angle=None, radians=False, cw=True):
        """ Calculates the change in angle between two encoder values. Both
            angles should be of the same unit (degree or radian).

           This method only support a differential drive system.
        :param ref_angle:
            The reference encoder values
        :param new_angle:
            The new encoder values
        :param radians:
            Flag to determine the unit of the angle.
        :param cw:
            Flag used to determine turnover
        :return:
            The change in angle between the two encoder values.
        """
        if new_angle is None:
            new_angle = self.read_encoders()

        diff = {}
        # Only add the difference of the keys that match the value of encoder
        # left or encoder right.
        for dist in ref_angle:
            if dist in new_angle:
                forward = None

                # Note for Differential Drive:
                # CW = enc_L -> Forward & enc_R -> Backward
                # CCW = enc_L -> Backward & enc_R -> Forward

                if dist == Drive.ENCODER_L:
                    forward = not cw
                elif dist == Drive.ENCODER_R:
                    forward = cw

                if forward is not None:
                    diff[dist] = Robot._encoder_diff(ref_angle[dist],
                                                     new_angle[dist],
                                                     forward)

        if len(diff) != 2:
            print "Not enough encoder values provided to calculate angle for" \
                  " a differential drive system."
            return 0

        angle = (diff[Drive.ENCODER_L] - diff[Drive.ENCODER_R]) / WHEEL_BASE

        if radians:
            return angle
        else:
            # Formula Source:
            #   x(in degrees)/y(in radians) = 360/2pi -> x = y*180*pi
            return angle*180/math.pi

    # ----------------------- #
    # -   Private Helpers   - #
    # ----------------------- #

    @staticmethod
    def _convert_bound(value, lower_bound, upper_bound):
        """ This begins by converting the provided value into a 16 bit two's
            complement integer. Next, it bounds the converted integer between
             the provided upper and lower bounds.

        :param value:
            The value to bound.
        :param lower_bound:
            The minimum value this value can possesses.
        :param upper_bound:
            The maximum value this value can possesses.
        :return:
            The bounded version of the value which is based on the upper and
            lower bounds.
        """
        # Converts value to 16 bit two's complement integer via bitwise.
        most_sig_bit = 0x8000

        # Gets the two least significant bits
        convert_val = value & _BYTE << _BYTE_SIZE | value & _BYTE
        # Extends the most significant bit if it is a 1. This is done by
        # carrying out the most significant bit.
        if bool(convert_val & most_sig_bit):
            convert_val |= ~(_BYTE << _BYTE_SIZE | _BYTE)

        # Bounds the converted value
        if convert_val > upper_bound:
            return upper_bound
        elif convert_val < lower_bound:
            return lower_bound
        return convert_val

    def _read_packet(self, packet_id, data_bytes):
        """ Sends the sensor command with the provided packet id to the robot
            and reads the robots response.

        :param packet_id:
            The packet id of the desired packet
        :param data_bytes:
            The number of bytes the robot will respond with
        :return:
            The raw data that the robot responded with.
        """
        self._serial_conn.send_command(_SENSORS_OPCODE+" "+str(packet_id))
        return self._serial_conn.read_data(data_bytes)

    def _read_encoder_raw(self, packet_id):
        data = self._read_packet(packet_id, Drive.ENCODER_BYTES)

        if len(data) == Drive.ENCODER_BYTES:
            return struct.unpack(">h", data)[0]
        else:
            return 0

    @staticmethod
    def _encoder_diff(ref_count, new_count, forward=True):
        """ Calculate the difference between two encoder distances. This will
            consider the turnover.

        :param ref_count:
            The reference distance of an encoder
        :param new_count:
            The new count of an encoder
        :param forward:
            Flag to determine which case to check
        :return:
            The difference in encoder distances.
        """
        if forward and ref_count > new_count:
            return (Drive.MAX_DIST - ref_count) - (Drive.MIN_DIST - new_count)
        elif not forward and new_count > ref_count:
            return (Drive.MIN_DIST - new_count) - (Drive.MAX_DIST - ref_count)
        else:
            return new_count - ref_count