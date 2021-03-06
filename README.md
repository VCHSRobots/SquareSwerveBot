# SquareSwerveBot

This repository is for Swerve Code written from scratch for the Square Swerve Bot that team 4415, Epic Robots, built in Fall 2019.  The Square Swerve Bot is an experiment to learn how to make and program swerve bots.

## Robot Configuration
The wheels are arranged in a square configuration, where the distance between adjacent wheels is 14 inches, measured to where the wheels touch the ground. The wheels are 4 inches in diameter.

A Neo motor, connected to a Spark-Max controller is used for driving. The gears between the wheel and the motor are 12:36 then 18:36, giving a 1:6 ratio between the Neo and the Wheels.

Steering is done with a 775 motor and a VersaPlanetary gear box that interfaces with a 132 tooth ring through a 36 tooth spur gear. The VersaPlanetary gear box is configured for 50:1. Therefore, the total gear reduction from the 775 to the steering ring is (132/36)*50 = 1:183.333.

An encoder is placed on the output stage of the VersaPlanetary gear box.  Therefore, there are (132/36) rotations on the
encoder for every one rotation of the steering ring. The encoder produces 4096 ticks per rotation, so it produces 15018.66666 ticks per one rotation of the steering ring.

There is a Hall Effect Sensor located at the diagonal of each swerve unit.  This sensor reads "true" for one position of
the steering ring, and false for all other positions. Therefore this sensor can be used to move the steering ring to a
known position.  However, because each unit is installed with a different orientation in respect to the robot, each
offset from the known position is different for each unit.

The Hall Effect sensor is connected to the drive motor's controller -- not to the steering motor controller, which would be more intuitive.

Please see RobotMap.java for IDs of the controllers and angle offsets for each swerve unit.

## Software Organization
This code is a Command Based Robot.  The core of the swerve drive related code is housed in SwerveUnit.java -- which provides control over one swerve unit.  Four of these are instantiated in the subsystem SwerveDriveSubSystem.  

There are various commands, where the more significant ones are: 

 * AdvancedDriveCommand            -- True 3 degress of freedom driving.
 * MoonDriveCommand                -- Orbits an object while keeping the front of the robot facing the object.
 * BasicDriveCommand               -- First try at drive control -- doesn't work very well.
 * CalibrateSwerveSystemCommand    -- For Calibration of the swerve units at start up. 

The actual calibration sequence is contained in CalibratedSwerveUnitCommand.  Four of these are called on by CalibrateSwerveSystemCommand.

As is the pattern of a command based robot, there is OI.java which ties the commands, joysticks, inputs, and subsystems together.  And there is RobotMap.java which defines the hardware configuration of the bot.

## PID Loops
All motors are controlled by PID loops so that they can be commanded with human understood units.  The steering rings on the swerve units can be set to spin at a given RPM, or to a absolute angle, in degrees, referenced to the front of the robot.  The steering rings can spin faster than 360 degrees per second. The Drive motors are commanded in units of feet per second.

## How To Drive
The current drive offers true 3-degrees of freedom.  The X and Y of the joystick are used to indicate direction and speed.  Just point and go.  The Z axis (Twist) will contorl the robot's rotation about it's center, even while moving.

Testing was done with a Logitech Extreme 3D joystick which provides X, Y, Twist, and Trim paddles, as well as 12 buttons. See OI.java to reconfigure for a different joystick or game paddle.  It should be easy to convert the code for use with PS3 and/or XBox game pads.

Here is the current mappings of the buttons:

  * Button 1:  Switch to moon drive mode -- while held down (see below).
  * Button 2:  Switches the view of the tabs on the shuffle board.
  * Button 3:  Strafe Drive -- while held down (see below).
  * Button 7:  Performs a "fast" ("loose") calibration of the swerve units.
  * Button 8:  Performs a "tight" calibration of the swerve units.
  * Button 10:  Performs a "tight" calibration on the Front Right unit only.
  * Button 11:  Spins the Front Right steering wheel at 100 rpm.

## Moon Mode
The robot can "orbit" a fixed point while continually facing the point, much like the moon orbits the earth.  To use this mode, do the following:

    1. Drive the robot towards the object to be orbited, until the object is touched.
    2. Press the trigger button, and hold it down during the next steps.
    3. Back away from the object, until you reach to the oribital distance.
    4. Push forward on the stick, past zero, and then select a direction of orbit by moving the stick right or left.
    5. When the trigger is released, normal driving is resumed.

The orbit mode assumes the object is 12 inches in diameter.  If it is larger, then it won't be possible (without modification to this mode)
to establish an exact orbit.  

## Strafe Drive Mode
Someday this will cause the robot to only move directions that are parallel to is't frame, or parallel to the field.  For now, it just
disabled the "twist", so that the robot will only move and not rotate around its axis.

This command is active while button 3 is being pressed.

## Swerve Calibration

Before the robot can operate, the swerve units must be calibrated.  This involves spinning the steering wheels to find known positions, and then resetting the encoders.  

There are two types of calibration: a "loose" and "tight".  The loose calibration is faster; it spins the steering wheel at a high rate to find the known position.  This results in some measurement error.  The "tight" calibration, first does a loose calibration, and then backs off the known position and approaches it a second time at slower rate, to get a better measurement.  

When the robot enters Autonomous or TeleOperated and the swerve drive is uncalibrated, the bot automatically performs a loose calibration.  Once the swerve is calibrated the robot can go in and out of the enabled modes and remain calibrated. However, if new code is loaded, or power is cycled, or the RoboRio is reset, the calibration is lost.

In a competitive environment, it would be important to perform a tight calibration before the match and then keep the robot powered on.  However, in the event that power is cycled, all is not lost -- only about 4 seconds after the robot is first enabled for either Autonomous or TeleOperated.

## ShuffleBoard Integration
This code features shuffleboard integration by extending most major classes to be a "SendableBase".  This means the resulting object has a name, and usually overrides "initSendable" to send the objects data to the shuffleboard.  Note that a Command and a Subsystem are already derived from SendableBase.  Once a class is a SendableBase, then lots of internal variables can be sent to the suffleboard as a group.  Note also, that objects that are SendableBase can be placed on designated tabs at designated places.
 
In this code, RobotMap.java is used to set the locations of UI elements for each of the major classes.  

## Testing System
This code also features a pattern for test code.  See the package frc.robot.tests.   If you derive from "TestBase", and then add your new class to "TestManager" then you can easily integrate tests into a menu system that works with shuffleboard. 

![Square Swerve Bot](https://github.com/VCHSRobots/SquareSwerveBot/blob/master/PicSwerveBot2019.jpg)

