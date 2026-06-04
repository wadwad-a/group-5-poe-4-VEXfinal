# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Authors:      Aaron D'Souza & Aaron Okon                                   #
# 	Created:      6/1/2026, 1:59:21 PM                                         #
# 	Description:  Code for VEX RECbot final                                    #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()

# ---------- Robot Configuration ---------------------------------------------------------
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)  # Right drivetrain motor
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)    # Left drivetrain motor
driveTrain = DriveTrain(leftMotor, rightMotor)                  # Run both motors simultaneously
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)   # Lift motor
inertial_1 = Inertial(Ports.PORT5)                              # Inertial sensor
liftArmRotation = Rotation(Ports.PORT6, False)                  # Liftarm rotation sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                    # Bumper switch
# ----------------------------------------------------------------------------------------

# ---------- Helper Functions ------------------------------------------------------------
def bump():
    """
    Hold the program's execution until the button is pressed.
    """
    while bumpSwitch.pressing() == False:
        wait(10, MSEC)  # Debounce the button

        brain.screen.set_cursor(1, 1)
        brain.screen.print("Press the button to start the program.")
        pass
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Program executed.")
    wait(1, SECONDS)

def inertialCalibration():
    """
    1. Calibrate the inertial sensor.
    2. Wait 2 seconds for calibration
    3. Call to start program execution
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Calibrating inertial sensor")
    brain.screen.set_cursor(2, 1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate() # calibrate the inertial sensor
    wait(2, SECONDS)       # wait for 2 seconds to allow the sensor to calibrate

    brain.screen.set_cursor(1, 1)
    brain.screen.clear_line(1)
    brain.screen.print("Inertial sensor calibrated.")

def inertialTest():
    """
    1. Test the inertial sensor by printing the current heading and rotation data to the screen
    2. Press the button to end the test
    """
    brain.screen.clear_screen()
    while bumpSwitch.pressing() == False:
        wait(10, MSEC)  # Debounce the button
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Heading: " + str(inertial_1.heading()))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Rotation: " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8, 1)
        brain.screen.print("Press the button to end the test.")

def driveStraightData(e):
    """
    1. Report position, rotation, and error
    2. Parameter: e = error value (setpoint - rotation)
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Position: " + str(leftMotor.position()))    # Return current encoder count

    brain.screen.set_cursor(2, 1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation()))   # Return current rotation

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Error: " + str(e))                          # Return current error

def stopMotors():
    """
    Stop both motors at the same time.
    """
    driveTrain.stop()
    wait(0.5, SECONDS)  # Wait 0.5 seconds for the system to stabilize

def accelerate(currentVelocity, targetVelocity, accelerationRate):
    """
    1. Gradually accelerate the motors from the current velocity to the target velocity at a specified acceleration rate.
    2. Parameters:
        - current_velocity: The current velocity of the motors (in percentage).
        - target_velocity: The desired velocity of the motors (in percentage).
        - acceleration_rate: The rate at which to accelerate (in percentage per second).
    """
    if currentVelocity >= targetVelocity:
        currentVelocity = targetVelocity        # If current velocity exceeds target velocity, set current velocity to target velocity
    else:
        currentVelocity += accelerationRate     # Otherwise, increase current velocity by the acceleration rate

    return currentVelocity

def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance in inches
    2. setpoint = 0-degrees for driving straiight
    3. motorVelocity = nominal motor velocity (+) => forward, (-) => backward
    """
    currentVelocity = 0    # Initialize current velocity for acceleration control

    inertial_1.reset_rotation()  # Reset the rotation value before taking action

    # Set stopping mode for motors
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    kP = 0.53   # Proportional constant for driving straight
                # Used to calculate the correction to maintain course
                # If too small, correction will occur too slwoly
                # If too large, overcorrection will occur
                # Determine best value by iteratively testing

    wheelDiameter = 4.0                             # 4" wheel diameter
    wheelCircumference = wheelDiameter * math.pi    # Calculate the wheel circumference

    # Convert the distance in inches to distance in ticks
    # distance (ticks) = (distance (inches) / wheel circumference) * 360 ticks per revolution
    distance = (distance / wheelCircumference) * 360

    # Reset the motor encoders
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    # Drive forward if motorVelocity > 0
    if motorVelocity > 0:
        while leftMotor.position() < distance:
            currentVelocity = accelerate(currentVelocity, motorVelocity, 0.1)  # Adjust acceleration rate as needed
            e = setpoint - inertial_1.rotation()    # Error
            correction = kP * e                     # Motor velocity correction

            # Correct motor velocities
            # if e > 0, (setpoint > rotation) => drifting left
            # if e < 0, (setpoint < rotation) => drifting right
            
            leftMotor.set_velocity(currentVelocity + correction, PERCENT)
            rightMotor.set_velocity(currentVelocity - correction, PERCENT)

            # Spin the motors
            driveTrain.drive(FORWARD)   # Drive both motors forward

            driveStraightData(e)    # Display position, rotation, and error
        stopMotors()    # Stop both motors when the desired distance is reached
    
    else:
        distance *= -1    # distance count negative
        while leftMotor.position() > distance:
            e = setpoint - inertial_1.rotation()    # Error
            correction = kP * e                     # Motor velocity correction

            # Correct motor velocities
            # if e > 0, (setpoint > rotation) => drifting left
            # if e < 0, (setpoint < rotation) => drifting right
            
            leftMotor.set_velocity(motorVelocity + correction, PERCENT)
            rightMotor.set_velocity(motorVelocity - correction, PERCENT)

            # Spin the motors
            driveTrain.drive(FORWARD)   # Drive both motors forward

            driveStraightData(e)    # Display position, rotation, and error
        stopMotors()    # Stop both motors when the desired distance is reached

def turnData(turnError, derivative):
    """
    1. Report heading, error, and derivative
    2. Parameter: e = error value (setpoint - rotation)
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) # Return current encoder count

    brain.screen.set_cursor(2, 1)
    brain.screen.print("Error: " + str(turnError))              # Return current rotation

    brain.screen.set_cursor(3, 1)
    brain.screen.print("Derivative: " + str(derivative))        # Return current error
    

def pointTurn(setPoint):
    """
    1.  Perform a point turn using the inertial sensor heading and proportional & derivative control
    2.  Argument: Desired Heading (setPoint)
    """
    brain.screen.clear_screen() # Clear screen

    # Set stopping mode for motors
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the difference between the setPoint and the current heading to determine turning direction
    difference = setPoint - inertial_1.heading()

    if setPoint > inertial_1.heading():
        if abs(difference) <= 180:
            clockwise = True;   # turn cw
        else:
            clockwise = False;  # turn ccw
    else:
        if abs(difference) <= 180:
            clockwise = False;  # turn ccw
        else:
            clockwise = True;   # turn cw

    # define kp and kd for CW and CCW turns
    if clockwise:
        kP = 0.04
        kD = 0.07
    else:
        kP = 0.04
        kD = 0.015
    

    # define maximum turning velocity and previous error term
    maxVelocity = 50
    previousError = 0.0


    while True:
        turnError = setPoint - inertial_1.heading()     # Calculate the error term
        derivative = turnError - previousError          # Calculate the derivative term

        if abs(turnError) < 1 and abs(derivative) < 0.2:   # If the error and derivative are both small, the turn is complete
            stopMotors()    # Stop the motors
            break           # Exit the loop

        # Calculate the correction for the motor velocities
        turnCorrection = (kP * turnError) + (kD * derivative)

        # Limit the turn correction between -1 and positive 1
        if abs(turnCorrection) > 1:
            turnCorrection = 1
        
        turnVelocity = maxVelocity * turnCorrection    # Calculate the turning velocity

        # Set the motor velocities for a point turn
        if clockwise:
            leftMotor.set_velocity(turnVelocity, PERCENT)
            rightMotor.set_velocity(-1 * turnVelocity, PERCENT)
        else:
            leftMotor.set_velocity(-1 * turnVelocity, PERCENT)
            rightMotor.set_velocity(turnVelocity, PERCENT)

        # Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative)     # Display heading, error, and derivative
        previousError = turnError           # Update the previous error term for the next iteration
        wait(20, MSEC)                      # Wait for 20 milliseconds before the next loop iteration

def liftArm(motorVelocity, liftAngle):
    # Configure the motor to hold its position
    liftMotor.set_stopping(HOLD)

    liftMotor.set_velocity(motorVelocity, PERCENT)   # Set the motor velocity

    gearRatio = 5      # 60T to 12T
    motorAngularDisplacement = liftAngle * gearRatio    # calculate motor axle's angular displacement

    # Spin motor forward for the given angular displacement
    liftMotor.spin_for(FORWARD, motorAngularDisplacement, DEGREES)
    wait(0.5, SECONDS)  # Wait for 0.5 seconds to allow the motor to stabilize
# ----------------------------------------------------------------------------------------

# ---------- Main Function ---------------------------------------------------------------
def main():
    """
    The main() function is the program that is executed by the Brain.
    """
    bump()                  # begin program execution

    # Set stopping mode for left and right motors to reduce lurch
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)
    inertialCalibration()   # calibrate the inertial sensor


    driveStraight(94, 0, 90)        # drive forward to where the ball is located
    liftArm(20, 50)                 # lift the arm to grab the ball
    driveStraight(10, 0, -40)       # drive backward to align the position with the next turn
    pointTurn(90)                   # turn 90 degrees to the right to face the goal
    driveStraight(65, 0, 90)        # drive forward to the goal
    pointTurn(45)                   # turn 45 degrees to the right to align with the goal
    driveStraight(12, 0, 90)        # drive forward to the goal
    liftArm(20, -50)
# ----------------------------------------------------------------------------------------

main()


        
