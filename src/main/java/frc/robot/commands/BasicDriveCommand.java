// BasicDriveCommand.java -- Basic Swerve Drive with Joystick

// In this basic drive system, all wheels are tied together in 
// direction and speed.  The Y axis of the joystick controls
// the speed, and the X axis controls the turn rate.
//
// This drive system is incapable of spining the robot
// while moving in a streight line.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utils.AngleCals;

// Simple Joystick connection to Swerve Drive.
public class BasicDriveCommand extends Command {
  double m_heading = 0;
  double m_lastTime;
  double m_maxTurnRate = 180.0;  // Degrees per second at full joystick position.
  double m_maxSpeed = 15.0;      // Feet per second at full joystick position.

  public BasicDriveCommand() {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
  }

  @Override
  protected void initialize() {
    super.initialize();
    m_heading = Robot.m_swerve.getAverageHeading();
    m_lastTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double speed = Robot.m_oi.getY() * m_maxSpeed;
    double turn_rate = Robot.m_oi.getX() * m_maxTurnRate;
    double current_time = Timer.getFPGATimestamp();
    double telp = current_time - m_lastTime;
    m_lastTime = current_time;
    double turn_delta = telp * turn_rate;
    double turn_d;
    if (turn_delta > 0.0) {
      turn_d = Math.min(turn_delta, 30.0);
    } else {
      turn_d = Math.max(turn_delta, -30.0);
    }
    m_heading = AngleCals.clamp_angle(m_heading + turn_d);
    Robot.m_swerve.setSpeed(speed);
    Robot.m_swerve.setHeading(m_heading);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_swerve.stop();
  }
}