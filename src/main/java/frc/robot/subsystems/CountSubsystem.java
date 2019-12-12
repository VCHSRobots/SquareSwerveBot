// CountSubsystem.java -- A Simple Counter for testig SmartDashboard/ShuffleBoard

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.PIDBase;

public class CountSubsystem extends Subsystem {

  private int m_Counter = 0;
 
  public CountSubsystem() {
    m_Counter = 0;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(null);
  }

  public void Count() {
    m_Counter += 1;
  }

  public void Reset() {
    m_Counter = 0;
  }

  public int getCount() {
    return m_Counter;
  }

  public void setCount(double newvalue) {
    m_Counter = (int) newvalue;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Count", this::getCount, this::setCount);
  }
}