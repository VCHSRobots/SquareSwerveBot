// DriveSetSpeedCommand.java -- Drive at the current direction with a set speed.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

// Simple Joystick connection to Swerve Drive.
public class DriveSetSpeedCommand extends Command {
  private double m_rate;
  public DriveSetSpeedCommand(double rate) {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
    m_rate = rate;
  }

  @Override
  protected void initialize() {
    super.initialize();
    System.out.printf("Starting DriveSetSpeedCommand at %5.2f feet/sec.\n", m_rate);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_swerve.setSpeed(m_rate);
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