CSCE 274 Section 1
Project 03
Group 3: Boyd Compton, Timothy Senn, & Jose Tadeo

-----------
- main.py -
-----------
    Dependences:
        - pySerial
        - robot_inf.py
        - sensor_inf.py
        - serial_inf.py

        PySerial will need to be installed on the system while robot_inf.py,
        sensor_inf.py and serial_inf.py just need to be placed in the same
        directory as main.py.

    Description:
        Main.py is the main program for project 3. This program was designed
        with the iRobot Create 2 in mind. The program will begin by initializing
        the robot's state to PASSIVE mode and then SAFE mode. After this, the
        robot will respond to the clean/power button being pressed.

        When the clean/power button is pressed and the robot is stopped, it
        begin to begin to follow a wall by using a PID controller. This PID
        controller will also handle right turns by simply adjusting the gains.
        Whenever a left turn is detected, the robot will then rotate in place
        counter-clockwise until no forward wall is detected.
        
        In the instance, when the robot cannot find a wall, it will move 
        forward until it detects an wall like obstacle. If the robot every 
        collides with an object, it will rotate in place for a random amount 
        of time. The left bumper will cause clockwise rotation while the right 
        bumper will cause counter-clockwise rotation. When both are pressed, 
        the robot will choose to rotate clockwise.
        
        Alternatively, if the robot is moving when the button is pressed, it
        will stop by stopping the daemon controlling the actuators in a safe
        manner.

    Execution:
        While in the directory with main.py, issue the command:

            python main.py

        The main program is now started. Once the text "Listening for press"
        is printed to the console, you can start the robot by pressing the
        clean/power button. Alternatively, the LED around the clean/power button
        will turn off when it is ready for input. Stop the robot by pressing
        the clean/power button again. To stop the execution of main.py,
        first, ensure the robot is stopped. Next, type in 'exit' into the console
        and then hit enter.
        
    Additional Help:
        For additional help, import the main.py module and issue the python
        help function. Example as shown:

            import main
            help(main)

----------------
- robot_inf.py -
----------------
    Dependences:
        - pySerial
        - serial_inf.py

        PySerial will need to be installed on the system while serial_inf.py
        just need to be in the same directory as robot_inf.py.

    Description:
        Robot_inf.py is an interface tailored specifically to the iRobot
        Create 2. In doing so, this module also contains specification for its
        OI such as:
            - Default Buad rate
            - Period for the sensor's update
            - States (Ex: START, PASSIVE, SAFE, RESET, STOP)
            - Buttons (Ex: CLEAN, DOCK)
            - Special values specific to Create 2 (Ex: SRAIGHT, TURN_CW, ENCODER_L)
            - Bump (Ex: BUMP_L, BUMP_R)
            - Light Bump (Infrared sensor) (Ex: LIGHT_BUMP_L)
            - WheelDrop (Ex: WHEEL_DROP_L)
            - Cliff (Ex: CLIFF_FL, CLIFF_R)

        Also, the interface contains methods that will:
            - Drive the robot using velocity and radius or velocity for each wheel
            - Change the robot's state
            - Set and play warning song
            - Read an individual button's or all buttons' values
            - Read an individual bump's or all bumps' values
            - Read an individual light bump's or all light bumps' values
            - Read an individual wheel drop's or all wheel drops' values
            - Read an individual cliff's or all cliffs' values
            - Read an individual encoder's or all encoders' values
            - Calculate the distance and angle from reference frame.

    Implementation:
        To use any of the constants available in the robot_inf.py module,
        reference them like so:

            robot_inf.<constant>

        For example, the clean button would be referenced like:

            robot_inf.Button.CLEAN

        To connect to an iRobot Create 2, instantiate an instance of the
        Robot class like so:

            create2 = robot_inf.Robot(serial_port)

        After creating the object, you have access to the following methods:
            - Change State (Ex: create2.change_state(robot_inf.State.SAFE))
                NOTE: * Only the values found in "robot_inf.State"
                        should be used as arguments.

            - Drive (Ex: create2.drive(velocity, radius)
                NOTE: * The method will only work in SAFE or FULL modes.
                      * That velocity is in mm/s and radius is in mm.
                      * Velocity ranges from -500 mm/s to 500 mm/s while
                          radius ranges from -2000 mm to 2000 mm.
                      * The special codes found in "robot_inf.Drive" are
                          values for the radius argument.

            - Read button (Ex: create2.read_button(robot_inf.Buttons.CLEAN))
                NOTE: * Only the values found in "robot_inf.Button" should
                        be used as arguments

            - Read buttons (Ex: btns = create2.read_buttons())
                NOTE: * This will return a dictionary that can be addressed
                        with values from "robot_inf.Button"
                       (Ex: btns[robot_inf.Buttons.CLEAN]).

            - Read bumps (Ex: create2.read_bumps())
                NOTE: * This will return a dictionary that can be addressed
                          with values from "robot_inf.Bump"
                          (Ex: bumps[robot_inf.Bump.BUMP_L]).
                      * Alternatively you can read individual drops by calling
                        create2.read_bump(<Bump>)
            
            - Read light bumps (Ex: create2.read_light_bumps())
                NOTE: * This will return a dictionary that can be addressed
                          with values from "robot_inf.Bump"
                          (Ex: bumps[robot_inf.Bump.LIGHT_BUMP_L]).
                      * Alternatively you can read individual drops by calling
                        create2.read_light_bump(<Light Bump>)

            - Read wheel drops (Ex: create2.read_wheel_drops())
                NOTE: * This will return a dictionary that can be addressed
                          with values from "robot_inf.WheelDrop"
                          (Ex: drops[robot_inf.WheelDrop.WHEEL_DROP_L]).
                      * Alternatively you can read individual drops by calling
                          create2.read_wheel_drop(<Wheel drop>)
                        
            - Read cliffs (Ex: create2.read_cliff(robot_inf.Cliff.CLIFF_R))
                NOTE: * This will return a dictionary that can be addressed
                          with values from "robot_inf.Cliff"
                          (Ex: cliffs[robot_inf.Cliff.CLIFF_R]).
                      * Alternatively you can read individual drops by calling
                          create2.read_cliff(<Cliff>)
                        
            - Read encoders (Ex: create2.read_encoders())
                NOTE: * This will return a dictionary that can be addressed
                          with values from "robot_inf.Drive.ENCODER_<L/R>"
                          (Ex: encs[robot_inf.Drive.ENCODER_R]).
                      * Alternatively you can read individual drops by calling
                          create2.read_encoder(<Encoder>)
            
            - Calculate distance or angle (Ex: create2.distance(ref_dist))
                NOTE: * 'ref_dist' is just the return from create2.read_encoders()

    Additional Help:
        For additional help, import the robot_inf.py module and issue the
        python help function. Example as shown:

            import robot_inf
            help(robot_inf)

-----------------
- sensor_inf.py -
-----------------
    Dependences:
        - pySerial
        - robot_inf.py

        PySerial will need to be installed on the system while robot_inf.py
        just needs to be in the same directory as sensor_inf.py.

    Description:
        Sensor_inf.py is an interface made for the sensors installed on the
        iRobot Create 2. This module contains race free methods that will:
            - Start and stop sensor data updater
            - Check if specified buttons have been pressed/released or are down
            - Check if the specified bumps are currently bumped
            - Check the specified light bumps' value
            - Check if the specified wheel drops are currently dropped
            - Check if there is a cliff on the specified side
            - Calculate the angle or distance
            - Get the encoders' value

        These methods are called within main in order to check the bumps,
        wheel drops, and cliffs read in by the robot's sensors.

    Implementation:
        To use sensor_inf.py, begin by importing:

            import sensor_inf

        If this import fails, ensure that sensor_inf.py is in the current
        working directory. This module gives access to the following methods
        once you have instantiated a Sensor object:
            - Determine if a button is being pressed
                (Ex: sensor.is_btn_pressed(robot_inf.Button.CLEAN))

            - Determine if a button has been released
                (Ex: sensor.is_btn_released(robot_inf.Button.CLEAN))

            - Determine if a button is currently down
                (Ex: sensor.is_btn_down(robot_inf.Button.CLEAN))

            - Determine if a bump is currently bumped
                (Ex: sensor.is_bump(robot_inf.Bump.BUMP_L))
                
            - Determine if a light bump is currently bumped
                (Ex: sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_L))

            - Determine if a wheel drop is currently dropped
                (Ex: sensor.get_wheel_drops())

            - Get cliff and wall sensor data
                (Ex: sensor.get_cliffs())
            
            - Get the encoders' value
                (Ex: sensor.get_encoders())
            
            - Get the distance or angle from reference point
                (Ex: sensor.get_distance(ref_dist) 
                     or sensor.get_angle(ref_angle))
                NOTE: The reference point is just the encoders value at that
                      point in time (sensor.get_encoders()).

    Additional Help:
        For additional help, import the sensor_inf.py module and issue the
        python help function. Example as shown:

            import sensor_inf
            help(sensor_inf)

-----------------
- serial_inf.py -
-----------------
    Dependences:
        - pySerial

        PySerial will need to be installed on the system.

    Description:
        Serial_inf.py is a generic interface for a serial connection. The
        interface will:
            - Establish a serial connection
            - Close a serial connection
            - Send space delimited commands as encoded ASCII commands (race free)
            - Read raw data (race free)
            - List available serial ports
        A serial connection can specify its serial port, buad rate, and read
        time out. However, the read time out will default to one second if not
        specified when establishing the connection. If you want to have no
        read time out, specify the time out as "None".

        When reading data with a read time out, the number of bytes returned
        can be less than the requested number of bytes.

    Implementation:
        To use serial_inf.py, begin by importing:

            import serial_inf

        If this import fails, ensure that serial_inf.py is within the current
        working directory. After this, you can get the available list of serial
        ports by issuing the following function:

            ports = serial_inf.list_serial_ports()

        This will return an array of strings that specify the available serial
        ports. To create a new serial connection, instantiate an instance of the
        SerialConn class like:

            conn = serial_inf.SerialConn(ports[0], buadrate, timeout=timeout)

        Note that the buadrate and timeout are variables declared prior to this
        call. Buadrate is an integer while timeout is a float. After this, you
        can do the following:
            - Send commands (Ex: conn.send_command('142 18'))
            - Read data (Ex: conn.read_data(num_of_bytes))
            - Close the connection (Ex: conn.close())
                NOTE: * The connection will automatically close when the
                        object gets destroyed.

        Additional Help:
            For additional help, import the serial_inf.py module and issue the
            python help function. Example as shown:

                import serial_inf
                help(serial_inf)
