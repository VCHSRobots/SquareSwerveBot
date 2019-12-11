// CalibrateSwerveSystemCommand.java -- Calibrate the entire swerve drive system.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.CalibrateSwerveUnitCommand.CalibrationType;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

// Calibrate all four Swerve Units.  
public class CalibrateSwerveSystemCommand extends Command {

  public CalibrateSwerveUnitCommand m_FL;
  public CalibrateSwerveUnitCommand m_FR;
  public CalibrateSwerveUnitCommand m_BL;
  public CalibrateSwerveUnitCommand m_BR;
  private boolean m_isFinished = false;
  private double m_startTime;
  
  public CalibrateSwerveSystemCommand(CalibrationType ctype) {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
    m_FL = new CalibrateSwerveUnitCommand(UnitID.kFrontLeft, ctype);
    m_FR = new CalibrateSwerveUnitCommand(UnitID.kFrontRight, ctype);
    m_BL = new CalibrateSwerveUnitCommand(UnitID.kBackLeft, ctype);
    m_BR = new CalibrateSwerveUnitCommand(UnitID.kBackRight, ctype);
  }

  @Override
  protected void initialize() {
    super.initialize();
    System.out.printf("Starting System Calibration on Swerve Drive.\n");
    m_isFinished = false;
    m_startTime = Timer.getFPGATimestamp();  
    m_FL.initialize(); 
    m_FR.initialize(); 
    m_BL.initialize(); 
    m_BR.initialize();
  }

  @Override
  protected void execute() {
    int nRunning = 0;
    if (!m_FL.isFinished()) {
      m_FL.execute();
      nRunning += 1;
    }
    if (!m_FR.isFinished()) {
      m_FR.execute();
      nRunning += 1;
    }
    if (!m_BL.isFinished()) {
      m_BL.execute();
      nRunning += 1;
    }
    if (!m_BR.isFinished()) {
      m_BR.execute();
      nRunning += 1;
    }
    if (nRunning == 0) {
      m_isFinished = true;
    }
    double telp = Timer.getFPGATimestamp() - m_startTime;
    if (telp > 8.0) {
      System.out.printf("Timeout on Swerve Drive System Calibration.\n");
      System.out.printf("Unit Stages: %d, %d, %d, %d.\n", 
        m_FL.getStage(), m_FR.getStage(), m_BL.getStage(), m_BR.getStage());
      m_FL.kill();
      m_FR.kill();
      m_BL.kill();
      m_BR.kill();
    }
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
}