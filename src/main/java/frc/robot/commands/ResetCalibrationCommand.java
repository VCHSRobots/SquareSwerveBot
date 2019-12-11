// ResetCalibrationCommand.java -- Kills the calibration flags.  

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

public class ResetCalibrationCommand extends Command {
  private boolean m_isFinished = false;

  public ResetCalibrationCommand() {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_swerve.getUnit(UnitID.kFrontLeft).resetCalibration();
    Robot.m_swerve.getUnit(UnitID.kFrontRight).resetCalibration();
    Robot.m_swerve.getUnit(UnitID.kBackLeft).resetCalibration();
    Robot.m_swerve.getUnit(UnitID.kBackRight).resetCalibration();
    m_isFinished = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
