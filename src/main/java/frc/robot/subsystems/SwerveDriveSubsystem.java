// SwerveDriveSubsystem.java -- Full Swerve Drive Subsystem

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotMap;
import frc.robot.commands.AdvancedDriveCommand;
import frc.robot.utils.AngleCals;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class SwerveDriveSubsystem extends Subsystem {

  public enum UnitID {
    kFrontLeft(0), kFrontRight(1), kBackLeft(2), kBackRight(3);
		public final int value;
		UnitID(int value) {
      this.value = value;
    }
  }

  public SwerveUnit m_FL; 
  public SwerveUnit m_FR; 
  public SwerveUnit m_BL; 
  public SwerveUnit m_BR; 
  private double m_desired_speed;
  private double m_desired_heading;
  private String m_mode_drive = "None";
  private String m_mode_steering = "None";

  // Creates the system of Swerve Units that make up the SwerveDrive system.
  // Should only have one of these in the robot.
  public SwerveDriveSubsystem() {
    int islot = RobotMap.SlotZeroForSU.x;
    m_FL = new SwerveUnit("FL", islot + 0, 
      RobotMap.SU_FL_Offset, RobotMap.SU_FL_DriveMotor, RobotMap.SU_FL_SteeringMotor);
    m_FR = new SwerveUnit("FR", islot+ 2, 
      RobotMap.SU_FR_Offset, RobotMap.SU_FR_DriveMotor, RobotMap.SU_FR_SteeringMotor);
    m_BL = new SwerveUnit("BL", islot + 4, 
      RobotMap.SU_BL_Offset, RobotMap.SU_BL_DriveMotor, RobotMap.SU_BL_SteeringMotor);
    m_BR = new SwerveUnit("BR", islot + 6, 
      RobotMap.SU_BR_Offset, RobotMap.SU_BR_DriveMotor, RobotMap.SU_BR_SteeringMotor);
    setupDashboard();
  }

  // Report the condition of this subsystem.
  public void report() {
    m_FL.report();
    m_FR.report();
    m_BL.report();
    m_BR.report();
  }

  // Read a few inputs directly from dashboard (Bypass OI).
  public void readFromDashboard() {
    m_FL.readFromDashboard();
    m_FR.readFromDashboard();
    m_BL.readFromDashboard();
    m_BR.readFromDashboard();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new AdvancedDriveCommand());
  }

  // Returns the given swerve unit.
  public SwerveUnit getUnit(UnitID id) {
    if (id == UnitID.kFrontLeft) {
      return m_FL;
    }
    if (id == UnitID.kFrontRight) {
      return m_FR;
    }
    if (id == UnitID.kBackLeft) {
      return m_BL;
    }
    if (id == UnitID.kBackRight) {
      return m_BR;
    }
    return null;
  } 

  // Moves the robot, where x, and y give the direction
  // and magnitude of the direction to move toward, 
  // relative to the current orientation of the robot,
  // and twist gives a magnitude of how much to 
  // turn the robot in place.
  public void move(double x, double y, double twist) {
    y = y * 0.3;
    twist = twist * 0.3;
    m_FL.setDrivePercentage(y);
    m_FL.setSteeringPercentage(twist);
    m_FR.setDrivePercentage(y);
    m_FR.setSteeringPercentage(twist);
    m_BL.setDrivePercentage(y);
    m_BL.setSteeringPercentage(twist);
    m_BR.setDrivePercentage(y);
    m_BR.setSteeringPercentage(twist);
    m_desired_heading = 0.0;
    m_desired_speed = 0.0;
    m_mode_drive = "Percentage";
    m_mode_steering = "Percentage";
  }

  // Stops all motors on the swerve drive.
  public void stop() {
    m_FL.stop();
    m_FR.stop();
    m_BL.stop();
    m_BR.stop();
    m_desired_speed = 0.0;
    m_mode_drive = "None";
    m_mode_steering = "None";
  }

  // Sets all the motors to travel at a constanct speed, given in feet/sec.
  public void setSpeed(double rate) {
    m_FL.setDriveSpeed(rate);
    m_FR.setDriveSpeed(rate);
    m_BL.setDriveSpeed(rate);
    m_BR.setDriveSpeed(rate);
    m_desired_speed = rate;
    m_mode_drive = "RPM";
  }

  // Sets the heading of all the wheels, given as an angle in degrees, refereced
  // from the front of the robot.
  public void setHeading(double angle) {
    m_FL.seekToAngle(angle);
    m_FR.seekToAngle(angle);
    m_BL.seekToAngle(angle);
    m_BR.seekToAngle(angle);
    m_desired_heading = angle;
    m_mode_steering = "POS";
  }

  // Sets the speed of all the drive motors, in feet/sec.
  public void setSpeed(double fl, double fr, double bl, double br) {
    m_FL.setDriveSpeed(fl);
    m_FR.setDriveSpeed(fr);
    m_BL.setDriveSpeed(bl);
    m_BR.setDriveSpeed(br);
    m_desired_speed = 0.0;
    m_mode_drive = "RPM++";
  }

  // Sets the heading of all the wheels, given as an angle in degrees, refereced
  // from the front of the robot.
  public void setHeading(double fl, double fr, double bl, double br) {
    m_FL.seekToAngle(fl);
    m_FR.seekToAngle(fr);
    m_BL.seekToAngle(bl);
    m_BR.seekToAngle(br);
    m_desired_heading = 0.0;
    m_mode_steering = "POS++";
  }

  public boolean isCalibrated() {
    if (m_FL.isCalibrated() && m_FR.isCalibrated() &&
      m_BL.isCalibrated() && m_BR.isCalibrated()) return true;
    return false;
  }

  public boolean isTightCalibrated() {
    if (m_FL.isTightCalibrated() && m_FR.isTightCalibrated() &&
      m_BL.isTightCalibrated() && m_BR.isTightCalibrated()) return true;
    return false;
  }

  public double getAverageHeading() {
    double a1 = AngleCals.average(m_FL.getSteeringAngle(), m_FR.getSteeringAngle());
    double a2 = AngleCals.average(m_BL.getSteeringAngle(), m_BR.getSteeringAngle());
    return AngleCals.average(a1, a2);
  }

  public double getAverageSpeed() {
    double avg = 0.25*(m_FL.getSpeed() + m_FR.getSpeed() + m_BL.getSpeed() + m_BR.getSpeed());
    return avg;
  }

  public String getMode() {
    return m_mode_drive + " / " + m_mode_steering;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveDrive");
      builder.addStringProperty("Current Command", this::getCurrentCommandName, null);
      builder.addBooleanProperty("Calibrated Loose", this::isCalibrated, null);
      builder.addBooleanProperty("Calibrated Tight", this::isTightCalibrated, null);
      builder.addStringProperty("Mode", this::getMode, null);
      builder.addDoubleProperty("Heading Desired ", () -> m_desired_heading, null);
      builder.addDoubleProperty("Headging Average", this::getAverageHeading, null);
      builder.addDoubleProperty("Speed Desired", () -> m_desired_speed, null);
      builder.addDoubleProperty("Speed Average", this::getAverageSpeed, null);
  }

  public void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(RobotMap.MainTabName);
    int x = RobotMap.SlotZeroForSS.x;
    int y = RobotMap.SlotZeroForSS.y;
    tab.add(this).withPosition(x,y).withSize(3,3);
  }
}