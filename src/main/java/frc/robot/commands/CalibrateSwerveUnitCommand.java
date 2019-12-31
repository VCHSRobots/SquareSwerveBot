// CalibrateSwerveUnitCommand.java -- Calibrates a swerve Unit 

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveUnit;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

// Automatic calibration of a swerve unit.
public class CalibrateSwerveUnitCommand extends Command {

  public enum CalibrationType {
    kFast(0), kTight(1);
		public final int value;
		CalibrationType(int value) {
      this.value = value;
    }
  }

  private UnitID m_unitid;
  private CalibrationType m_ctype;
  private int m_istage;    // Stages of calibration. 0=not started, 1=first found
  private SwerveUnit m_unit; // The Unit under calibration
  private double m_starting_angle;
  private double m_starting_time;
  private double m_moving_rate;
  private boolean m_isFinished = false;

  public CalibrateSwerveUnitCommand(UnitID unitid, CalibrationType ctype) {
    requires(Robot.m_swerve);
    this.setInterruptible(false);
    m_unitid = unitid;
    m_ctype = ctype;
    m_unit = Robot.m_swerve.getUnit(m_unitid);
  }

  @Override
  protected void initialize() {
    m_isFinished = false;
    m_istage = 0;
    System.out.printf("CalibrateServeUnitCommand For Unit %s started.\n", m_unit.getName());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch(m_istage) {
      case 0: stage_0(); return;
      case 1: stage_1(); return;
      case 2: stage_2(); return;
      case 3: stage_3(); return;
      case 4: stage_4(); return;
      case 10: stage_10(); return;
      case 11: stage_11(); return;
    }
    m_isFinished = true;
    m_istage = 1000;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_isFinished;
  }

  // Returns the internal stage for debugging purposes.
  public int getStage() {
    return m_istage;
  }

  // Kill the operation before it's done.
  public void kill() {
    m_istage = 1000;
    m_isFinished = true;
    m_unit.stop();
    System.out.printf("CalibrateServeUnit Command For Unit %s killed.\n", m_unit.getName());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  private void print_error(String msg) {
    System.out.printf("** Calibration failure for unit %s. %s", m_unit.getName(), msg);
  }

  private void stage_0(){
    // Assume nothing. Start procedure here.
    m_isFinished = false;
    if (m_unit.isAtPrime()) {
      m_unit.setSteeringRate(0.0);
      m_istage = 2;
      return;
    }
    m_moving_rate = 100.0;
    m_unit.setSteeringRate(m_moving_rate);
    m_starting_angle = m_unit.getAbsoluteAngle();
    m_starting_time = Timer.getFPGATimestamp();
    m_istage = 1;
    return;
  }

  private void stage_1(){
    // At this point, the unit should be turning at a pretty fast rate to locate the prime point.
    if (m_unit.isAtPrime()) {
      m_unit.setSteeringRate(0.0);
      m_unit.reset_steering_encoder();
      m_unit.setCalibrated("Loose");
      if (m_ctype == CalibrationType.kFast) {
        m_istage = 10; // Jump to end, and skip tight calibration.
        return;
      }
      m_istage = 2; // Continue with tight calibration.
      return;
    }
    // Have we moved too far?  If so, slow down and keep trying.
    double delta = Math.abs(m_unit.getAbsoluteAngle() - m_starting_angle);
    if (delta > 450.0 && m_moving_rate >= 90.0) {
      m_moving_rate = 40;
      m_unit.setSteeringRate(m_moving_rate);
      System.out.println(String.format("Reducing Ring Movement Rate for Unit %s during Calibration.", m_unit.getName()));
    }
    // Has it been too long?
    double telp = Timer.getFPGATimestamp() - m_starting_time;
    if (telp > 6.0) {
      m_unit.setSteeringPercentage(0.0);
      print_error("Timeout in stage 1.");
      m_isFinished = true;
      m_istage = 100;
      return;
    }
  }

  private void stage_2(){
    // Here, the unit should be stopped, and near or on the prime point.
    // We now have a loose calibration. 
    // Back away, and get a better reading of where the prime point is.  
    m_starting_angle = m_unit.getAbsoluteAngle();
    m_starting_time = Timer.getFPGATimestamp();
    m_unit.setSteeringRate(-40.0);
    m_istage = 3;
  }

  private void stage_3(){
    // Here, the unit should be moving slowly in the reverse direction.
    // Wait for it to get 45 degrees away, then reverse it, and move
    // to the next stage.
    double angle_diff = Math.abs(m_unit.getAbsoluteAngle() - m_starting_angle);
    if (angle_diff > 30.0) {
      // We reached the turn around point.
      m_unit.setSteeringPercentage(0.0);
      if (m_unit.isAtPrime()) {
        print_error("Sensor problem. Reading AtPrime for over 45 degrees.");
        m_isFinished = true;
        m_istage = 100;
        return;
      }
      m_starting_angle = m_unit.getAbsoluteAngle();
      m_starting_time = Timer.getFPGATimestamp();
      m_unit.setSteeringRate(20.0);
      m_istage = 4;
      return;
    }
    // Has it been too long?
    double telp = Timer.getFPGATimestamp() - m_starting_time;
    if (telp > 4.0) {
      m_unit.setSteeringPercentage(0.0);
      print_error("Timeout in stage 3.");
      m_isFinished = true;
      m_istage = 100;
      return;
    }
  }

  private void stage_4(){
    // We should be moving slowing toward the prime point.
    if (m_unit.isAtPrime()) {
      m_unit.setSteeringRate(0.0);
      m_unit.reset_steering_encoder();
      m_unit.setCalibrated("Tight");
      m_istage = 10;
      return;
    }
    // Have we moved too far? 
    double delta = Math.abs(m_unit.getAbsoluteAngle() - m_starting_angle);
    if (delta > 50.0) {
      m_unit.setSteeringPercentage(0.0);
      print_error("In stage 4. Moved too far in slow mode without finding the prime point.");
      m_istage = 10;  // Go ahead and use loose calibration.
      return;
    }
    // Has it been too long?
    double telp = Timer.getFPGATimestamp() - m_starting_time;
    if (telp > 4.0) {
      m_unit.setSteeringPercentage(0.0);
      print_error("In stage 4. Timeout -- taking too long to move back to prime point.");
      m_istage = 10; // Go ahead and use loose calibration.
      return;
    }
  }

  private void stage_10(){
    // Here the unit should be stopped and Calibrated.  Move to the zero angle.
    m_starting_angle = m_unit.getAbsoluteAngle();
    m_starting_time = Timer.getFPGATimestamp();
    m_unit.seekToAngle(0.0);
    m_istage = 11;
  }

  private void stage_11() {
    // We should be moving to the final zero location.
    double angle_err = Math.abs(m_unit.getSteeringAngle());
    if (angle_err < 5.0) {
      m_istage = 100;
      m_isFinished = true;
      return;
    }
    // Has it been too long?
    double telp = Timer.getFPGATimestamp() - m_starting_time;
    if (telp > 2.0) {
      m_unit.setSteeringPercentage(0.0);
      print_error("Timeout on last move to zero degrees.");
      m_isFinished = true;
      m_istage = 100;
      return;
    }
  }

}