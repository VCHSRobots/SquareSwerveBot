// SteeringMotorTest.java -- tests and tunes the steering motor.

package frc.robot.tests;

import frc.robot.subsystems.SwerveUnit;
import frc.robot.Robot;

// Uses the Trim tab to set the angle to seek to.  Will only
// actually execute the seek when the joystick button 
// 1 is pressed.
public class SteeringMotorTest extends TestBase {

  SwerveUnit m_swerveunit;

  public SteeringMotorTest(String name, SwerveUnit swerveunit) {
    super(name);
    m_swerveunit = swerveunit;
  }

  @Override
  public void periodic() {
    if (!m_isrunning) return;
    if (Robot.m_oi.getRawButton(1)) {
      double angle = Robot.m_oi.getTrim() * 180.0;
      m_swerveunit.seekToAngle(angle);
    }
  }

  @Override
  public void shutdown() {
    m_swerveunit.stop();
    m_isrunning = false;
  }

}
