// TestManager.java -- Setups and manages all tests on the robot.

package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

// Sets up the tests used on the Robot.
public class TestManager {

  private SendableChooser<TestBase> m_chooser = new SendableChooser<TestBase>();
  private NetworkTableEntry m_ntentry_selectedTest;
  private NetworkTableEntry m_ntentry_runtest;
  private NetworkTableEntry m_ntentry_running;
  private TestBase m_currentTest = null;

  public TestManager() {
    SetupTestSelection();
  }

  public void SetupTestSelection(){
    m_chooser.setDefaultOption("Do Nothing", null);
    SwerveDriveSubsystem ss = Robot.m_swerve;
    m_chooser.addOption("Tune Steering Motor FL", new SteeringMotorTest("FL Steering", ss.getUnit(UnitID.kFrontLeft)));
    m_chooser.addOption("Tune Steering Motor FR", new SteeringMotorTest("FR Steering", ss.getUnit(UnitID.kFrontRight)));
    m_chooser.addOption("Tune Steering Motor BL", new SteeringMotorTest("BL Steering", ss.getUnit(UnitID.kBackLeft)));
    m_chooser.addOption("Tune Steering Motor BR", new SteeringMotorTest("BR Steering", ss.getUnit(UnitID.kBackRight)));

    ShuffleboardTab tab = Shuffleboard.getTab("Tests");
    tab.add("Test Selection", m_chooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withPosition(0,0).withSize(2,1);
    m_ntentry_selectedTest = tab.add("Selected Test", "None")
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0,1).withSize(2,1).getEntry();   
    m_ntentry_runtest = tab.add("Run Test", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(2,0).withSize(1,1).getEntry();  
      m_ntentry_running = tab.add("Running", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(2,1).withSize(1,1).getEntry();  
  }

  public void report() {
    TestBase sel = m_chooser.getSelected();
    if (sel == null) {
      m_ntentry_selectedTest.setString("None");
      m_ntentry_running.setBoolean(false);
    } else {
      m_ntentry_selectedTest.setString(sel.getName());
      m_ntentry_running.setBoolean(sel.isRunning());
    }
  }

  public void testPeriodic() {
    TestBase sel = m_chooser.getSelected();
    boolean doit = m_ntentry_runtest.getBoolean(false);
    // If the test has changed, then reset.
    if (sel != m_currentTest) {
      m_ntentry_runtest.setBoolean(false);
      if (m_currentTest != null) {
        m_currentTest.shutdown();
      }
      m_currentTest = sel;
      return;
    }
    if (!doit) {
      if (m_currentTest != null) {
        if (m_currentTest.isRunning()) {
          m_currentTest.shutdown();
        }
      }
      return;
    }
    if (m_currentTest != null) {
      if (!m_currentTest.m_isrunning) {
        m_currentTest.start();
      }
      m_currentTest.periodic();
    }
  }
}
