// TestBase.java -- Interface for tests that can be included in the test suite.

package frc.robot.tests;

// Use this interface to create an individule test to run on the robot.  
public abstract class TestBase {
  protected String m_name = "";
  protected boolean m_isrunning = false;
  protected boolean m_isAutomated = false;
  protected String m_status = "";

  public TestBase(String name) {
    m_name = name;
  }

  public Boolean isAutomated() {
    return m_isAutomated;
  }

  public String getName() {
    return m_name;
  }
  
  public void start() {
    m_isrunning = true;
  }

  public abstract void periodic();
  public abstract void shutdown();

  public String getStatus() {
    return m_status;
  }
  
  public Boolean isRunning() {
    return m_isrunning;
  }

}
