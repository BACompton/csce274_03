#!/usr/bin/python

import math
import random
import threading
import time


import robot_inf
import serial_inf
import sensor_inf


# =============================================================================
#                       Wall Following Controller
# =============================================================================

class WallFollow(threading.Thread):
    """
        This is the low level actuator controller to follow a wall with
        obstacle avoidance.

        :type _sensor sensor_inf.Sensor
        :type _stop bool
    """

    # -------------------------------------------------------------------- #
    # -                     Internal Constants                           - #
    # -------------------------------------------------------------------- #

    # The base velocity to apply to the wheels. This is the velocity of
    # forward movement in mm/sec.
    _WHEEL_VEL = 100

    # Maximum and minimum amount of time the robot can rotate when a bump
    # is pressed. Both limits are calculated from the general differential
    # drive angle formula solved for the time:
    #
    #   time  = wheel_base * angle_radians / ( 2 * wheel_velocity )
    #
    # The maximum wait time will rotate the robot roughly 15 degrees
    # The minimum wait time will rotate the robot roughly 5 degrees
    _MIN_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/12) / (2 * _WHEEL_VEL)
    _MAX_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/36) / (2 * _WHEEL_VEL)

    # The sensor value for the desired distance from the wall
    _WALL_GOAL = 250
    # The minimum distance the robot can be from a forward wall before turning
    # to the left.
    _LEFT_TURN_THRESHOLD = _WALL_GOAL - 150

    # The minimum sensor value required for a wall to be detected. This was
    # chosen based on the senor's noise.
    _WALL_THRESHOLD = 5

    # The maximum amount of cycles before a wall is considered lost. The
    # threshold's value is calculated based on the following formula:
    #
    #   num_of_cycles = max_time / (wait_time + avg_cycle_length)
    _LOST_WALL_THRESHOLD = math.ceil(15. / (robot_inf.SENSOR_UPDATE_WAIT + .25))

    # -------------------------------------------------------------------- #
    # -                       Class Definition                           - #
    # -------------------------------------------------------------------- #

    _sensor = None
    _stop = None

    def __init__(self, sensor):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._stop = False
        self._sensor = sensor

    # -------------------------------------------------------------------- #
    # -                        Main Behavior                             - #
    # -------------------------------------------------------------------- #

    def run(self):
        # The serial connection for the robot
        robot = self._sensor.get_robot()

        # The initial points for each PID controller
        init_follow_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_R)
        init_turn_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_CR)

        # The wait time in between behavior decisions
        wait_time = robot_inf.SENSOR_UPDATE_WAIT

        # PID controllers for the wall following/right turn behavior (PID) and
        # the left turn behavior (PD)
        wall_follow = robot_inf.PIDController(goal=WallFollow._WALL_GOAL,
                                              init_pt=init_follow_pt)
        left_turn = robot_inf.PIDController(goal=0, kP=2.5, kI=0, kD=0,
                                            init_pt=init_turn_pt)

        # The separate gains for the wall follow PID controller.
        #   - follow_gains: The default gains for the wall following behavior
        #   - right_turn_gains: Adjust the gains to perform a right wall follow
        #
        # Instead of creating a separate PID controller for right wall following,
        # the gains are adjusted to reduce the radius of a right turn since
        # the wall following PID controller already covers right turns.
        follow_gains = {
            robot_inf.PIDController.KP_KEY: .1,
            robot_inf.PIDController.KI_KEY: .0025,
            robot_inf.PIDController.KD_KEY: .0025
        }
        right_turn_gains = {
            robot_inf.PIDController.KP_KEY: .2
        }
        wall_follow.set_gains(follow_gains)

        # A counter for the number of cycles with no wall detected.
        no_wall_count = 0

        # ------------------------------------------------------------ #
        # -                    Behavior Decision                     - #
        # ------------------------------------------------------------ #

        while not self._wait(wait_time, robot_inf.SENSOR_UPDATE_WAIT):
            rotate_cw = self._turn_cw()
            rotate_ccw = self._turn_ccw()
            left_output = left_turn.get_output()

            # Get the current points for the PID controllers
            wall_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_R)
            left_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_CR)

            # If either bump is down, set wait time to a new random
            # wait time between the limits established above. Otherwise,
            # set the wait time to the sensor's response time
            if rotate_cw or rotate_ccw:
                wait_time = random.uniform(WallFollow._MIN_TURN_TIME,
                                           WallFollow._MAX_TURN_TIME)
            else:
                wait_time = robot_inf.SENSOR_UPDATE_WAIT

            # Decide which behavior to perform
            if rotate_cw:
                self._drive_cw()
            elif rotate_ccw:
                self._drive_ccw()
            elif left_output > WallFollow._LEFT_TURN_THRESHOLD:
                wall_follow.reset(init_pt=wall_pt)
                self._drive_ccw()
            elif no_wall_count >= WallFollow._LOST_WALL_THRESHOLD:
                no_wall_count = WallFollow._LOST_WALL_THRESHOLD
                self._drive_forward()
            else:
                self._wall_follow(wall_follow, left_turn)

            # Update the PID controllers with the current point
            left_turn.add_point(left_pt)

            # If no right wall
            if wall_pt < WallFollow._WALL_THRESHOLD:
                wall_follow.set_gains(right_turn_gains)
                no_wall_count += 1
                wall_follow.reset(init_pt=wall_pt)

            # Found right wall
            else:
                wall_follow.set_gains(follow_gains)
                no_wall_count = 0
                wall_follow.add_point(wall_pt)

        # Stops robot
        robot.drive_direct(0, 0)

    # -------------------------------------------------------------------- #
    # -                  Behavior Definitions                            - #
    # -------------------------------------------------------------------- #

    def _drive_ccw(self):
        """
            Rotates the robot counter-clockwise.
        """
        self._sensor.get_robot().drive_direct(WallFollow._WHEEL_VEL,
                                              -WallFollow._WHEEL_VEL)

    def _drive_cw(self):
        """
            Rotates the robot clockwise.
        """
        self._sensor.get_robot().drive_direct(-WallFollow._WHEEL_VEL,
                                              WallFollow._WHEEL_VEL)

    def _drive_forward(self):
        """
            Drives the robot forward.
        """
        self._sensor.get_robot().drive_direct(WallFollow._WHEEL_VEL,
                                              WallFollow._WHEEL_VEL)

    def _wall_follow(self, wall_pid, left_pid):
        """ The wall following module.

            This will follow a wall using a pid controller whose current
            position is based on an infrared sensor's reading.
        :type wall_pid robot_inf.PIDController
        :param wall_pid:
            The pid controller for the wall following and right turn behavior.
        :type left_pid robot_inf.PIDController
        :param left_pid:
            The pid controller for the left turn behavior.
        """
        # The maximum allowed output the left turn detection can be
        max_left_turn = int(3 * WallFollow._WHEEL_VEL)

        # PID controller output
        output = int(wall_pid.get_output())

        self._sensor.get_robot().drive_direct(WallFollow._WHEEL_VEL+output,
                                              WallFollow._WHEEL_VEL-output)

    # -------------------------------------------------------------------- #
    # -                      External Helpers                            - #
    # -------------------------------------------------------------------- #

    def stop(self):
        """
            Sets the flag to stop the thread to True. The thread will
            not immediately stop. Instead, the thread will exit safely.
        """
        self._stop = True

    # -------------------------------------------------------------------- #
    # -                      Internal Helpers                            - #
    # -------------------------------------------------------------------- #

    def _wait(self, wait_time, interval):
        """ Internal method that divides the time in between actuator
            commands into small intervals. This enables the ability
            to check for the stopping condition while running an action.

        :param wait_time:
            The total amount of time to wait
        :param interval:
            The amount of time for a single interval.
        :return:
            True if the stopping condition was detected, otherwise false.
        """
        time_left = wait_time

        # Special Case
        if time_left == 0:
            return self._should_stop()

        while time_left > 0:
            start = time.time()
            # Tell the caller it needs to stop
            if self._should_stop():
                return True

            # Wait another time interval
            interval_time = interval
            if interval_time > time_left:
                interval_time = time_left
            # time_left -= sleep time + elapsed time

            time_left -= (interval_time + (time.time()-start))
            time.sleep(interval_time)
        return False

    def _should_stop(self):
        """ Determines if the actuator controller should stop.
        :return:
            True if the actuator controller should stop.
        """
        return self._stop or not self._safe_motion()

    def _safe_motion(self):
        """ Determines if movement or rotation is safe.
        :return:
            True if motion or rotation is safe.
        """
        drops = self._sensor.get_wheel_drops()

        for drop in drops:
            if drops[drop]:
                return False
        return True

    def _turn_cw(self):
        """ Determines if the robot should rotate clockwise
        :return:
            True if the robot should turn clockwise
        """
        cliffs = self._sensor.get_cliffs()

        if self._sensor.is_bump(robot_inf.Bump.BUMP_L):
            return True
        if cliffs[robot_inf.Cliff.CLIFF_L] or cliffs[robot_inf.Cliff.CLIFF_FL]:
            return True
        return False

    def _turn_ccw(self):
        """ Determines if the robot should rotate counter-clockwise
        :return:
            True if the robot should turn counter-clockwise
        """
        cliffs = self._sensor.get_cliffs()

        if self._sensor.is_bump(robot_inf.Bump.BUMP_R):
            return True
        if cliffs[robot_inf.Cliff.CLIFF_R] or cliffs[robot_inf.Cliff.CLIFF_FR]:
            return True
        return False


# =============================================================================
#                       Button Press Listener
# =============================================================================

class RobotController(threading.Thread):
    """
        High level robot controller. This controller decides when to spawn and
        kill specific low level actuator controllers.
    """
    _stop = None

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._stop = False

    # -------------------------------------------------------------------- #
    # -                        Main Behavior                             - #
    # -------------------------------------------------------------------- #

    def run(self):
        # Get available serial connections
        port_list = serial_inf.list_serial_ports()

        # Check for at least one available serial connection
        if len(port_list) > 1:
            print "Requires a serial connection."
            return -1

        # Serial connection to robot.
        robot = robot_inf.Robot(port_list[0])
        robot.change_state(robot_inf.State.FULL)
        print "Connected to robot"

        # Synchronized sensor interface for the serial connection to the robot
        sensor = sensor_inf.Sensor(robot)

        # Current low-level actuator control
        act_control = None

        print "Listening for press"
        while not self._stop:
            # When the CLEAN button is pressed and the robot is stopped,
            # start to follow a wall. Otherwise stop the robot upon a CLEAN
            # button press.
            if sensor.is_btn_pressed(robot_inf.Button.CLEAN):

                # Start to wall follow
                if act_control is None or not act_control.isAlive():
                    act_control = WallFollow(sensor)
                    act_control.start()

                # Stop robot
                else:
                    act_control.stop()
                    act_control = None

            # Clocks while loop to the update rate of the iRobot Create 2.
            time.sleep(robot_inf.SENSOR_UPDATE_WAIT)

        print "Stopping Listening"

        # Cleans up all threads spawned during execution.
        robot.change_state(robot_inf.State.PASSIVE)
        if act_control is not None:
            act_control.stop()
        sensor.stop_update(join=True)

    # -------------------------------------------------------------------- #
    # -                      External Helpers                            - #
    # -------------------------------------------------------------------- #

    def stop(self):
        """
            Sets the flag to stop the thread to True. The thread will
            not immediately stop. Instead, the thread will exit safely.
        """
        self._stop = True


# =============================================================================
#                       Miscellaneous Helper Functions
# =============================================================================

def _bound(data, minimum=0, maximum=1):
    """
        Bounds some data between a max and minimum value.

        :param data:
            The data to be bounded
        :param minimum:
            The minimum value the data can have
        :param maximum:
            The maximum value the data can have
        :return:
            The bounded data
    """
    if data > maximum:
        return maximum
    if data < minimum:
        return minimum
    return data


# =============================================================================
#                               Main Function
# =============================================================================

def main():
    """
        The main loop that spawns and controls all the threads for the
        wall following project.
    """
    control = RobotController()
    control.start()

    # Prompt to exit safely
    while raw_input("Type 'exit' to quit.") != "exit":
        time.sleep(robot_inf.SENSOR_UPDATE_WAIT)

    control.stop()
    # Wait for every thing to stop safely
    control.join()

if __name__ == '__main__':
    main()
