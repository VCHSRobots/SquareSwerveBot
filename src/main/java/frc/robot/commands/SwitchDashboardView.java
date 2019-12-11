/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * An example command.  You can replace me with your own command.
 */
public class SwitchDashboardView extends Command {
  private int m_nView = 0;   // Current View on Shuffle Board
  private List<String> m_Views; 
  private Boolean m_Done = true;

  public SwitchDashboardView() {
    // Use requires() here to declare subsystem dependencies
    m_Views = new ArrayList<String>();
    m_Views.add("SmartDashboard");
    m_Views.add("LiveWindow");
    m_Views.add(RobotMap.MainTabName);
  }

   @Override
  protected void initialize() {
    m_Done = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("In SwitchView Command.");
    m_nView += 1;
    if (m_nView >= m_Views.size()) {
      m_nView = 0;
    }
    Shuffleboard.selectTab(m_Views.get(m_nView));
    m_Done = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_Done;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
