// CountCommand.java -- Counts one unit using the CountSubSystem.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CountCommand extends Command {
  private boolean m_isFinished = false;

  public CountCommand() {
    requires(Robot.m_counter);
    this.setInterruptible(true);
  }

  @Override
  protected void initialize() {
    m_isFinished = false;
    System.out.printf("CountCommand.initinalize called.\n");
  }

  @Override
  protected void execute() {
    Robot.m_counter.Count();
    m_isFinished = true;
  }

  @Override
  protected boolean isFinished() {
    return m_isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.printf("CountCommand.end called.\n");
  }
}
