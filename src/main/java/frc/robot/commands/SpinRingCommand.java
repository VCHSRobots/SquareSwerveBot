// SpinRingCommand.java -- Spin the steering ring at a given rate.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveUnit;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

// Spin the Steering Ring at a given rate.
public class SpinRingCommand extends Command {

  private UnitID m_unitid;
  private double m_rate;
  private SwerveUnit m_unit; // The Unit being spun

  public SpinRingCommand(UnitID unitid, double rate) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_swerve);
    this.setInterruptible(true);
    m_unitid = unitid;
    m_rate = rate;   
    m_unit = Robot.m_swerve.getUnit(m_unitid);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_rate = 360 * Robot.m_oi.getTrim();
    m_unit.setSteeringRate(m_rate);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_unit.setSteeringRate(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    m_unit.setSteeringRate(0.0);
  }
}
