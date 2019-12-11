// Robot.java -- Main robot code

// This software was writen in Fall 2019, to test a square swerve bot.
// Written by Mentor, Dal Brandon

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CalibrateSwerveSystemCommand;
import frc.robot.commands.CalibrateSwerveUnitCommand.CalibrationType;
import frc.robot.commands.ResetCalibrationCommand;
import frc.robot.commands.DriveSetSpeedCommand;
import frc.robot.commands.CountCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CountSubsystem;
import frc.robot.tests.TestManager;

public class Robot extends TimedRobot {
  public static SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();
  public static CountSubsystem m_counter = new CountSubsystem();
  public static OI m_oi = new OI();
  public static TestManager m_testManager = new TestManager();
  public int m_tpcount = 0;
  public CalibrateSwerveSystemCommand m_calibrateFastCommand = new CalibrateSwerveSystemCommand(CalibrationType.kFast);


  @Override
  public void robotInit() {
    // Put the subsystems on the dashboard.
    SmartDashboard.putData(m_counter);
    SmartDashboard.putData(m_swerve);
    // Put a few commands on the dashboard.
    SmartDashboard.putData("Count Once", new CountCommand());
    SmartDashboard.putData("Reset Swerve Calibration", new ResetCalibrationCommand());
    SmartDashboard.putData("Fast Calibration", new CalibrateSwerveSystemCommand(CalibrationType.kFast));
    SmartDashboard.putData("Tight Calibration", new CalibrateSwerveSystemCommand(CalibrationType.kTight));
    SmartDashboard.putData("Drive at 4 ft/sec", new DriveSetSpeedCommand(4.0));
  }

  @Override
  public void robotPeriodic() {
    m_swerve.readFromDashboard();
    m_swerve.report();
    m_testManager.report();
  }

  @Override
  public void disabledInit() {
    m_tpcount = 0;
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (!m_swerve.isCalibrated()) {
      m_calibrateFastCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (!m_swerve.isCalibrated()) {
      m_calibrateFastCommand.start();
    }
    Shuffleboard.selectTab(RobotMap.MainTabName);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
    if (m_tpcount == 0) {
      Shuffleboard.selectTab("Tests");
    }
    m_tpcount += 1;
    m_testManager.testPeriodic();
  }
}
