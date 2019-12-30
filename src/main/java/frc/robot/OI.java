// OI.java -- Binds Conrtrols to Commands

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.SendableBase;
import frc.robot.commands.*;
import frc.robot.commands.CalibrateSwerveUnitCommand.CalibrationType;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI extends SendableBase {
  private Joystick m_joystick = new Joystick(0);
  private final JoystickButton m_btn_1 = new JoystickButton(m_joystick, 1);
  private final JoystickButton m_btn_2 = new JoystickButton(m_joystick, 2);
  private final JoystickButton m_btn_3 = new JoystickButton(m_joystick, 3);
  private final JoystickButton m_btn_4 = new JoystickButton(m_joystick, 4);
  private final JoystickButton m_btn_5 = new JoystickButton(m_joystick, 5);
  private final JoystickButton m_btn_6 = new JoystickButton(m_joystick, 6);
  private final JoystickButton m_btn_7 = new JoystickButton(m_joystick, 7);
  private final JoystickButton m_btn_8 = new JoystickButton(m_joystick, 8);
  private final JoystickButton m_btn_9 = new JoystickButton(m_joystick, 9);
  private final JoystickButton m_btn_10 = new JoystickButton(m_joystick, 10);
  private final JoystickButton m_btn_11 = new JoystickButton(m_joystick, 11);
  private final JoystickButton m_btn_12 = new JoystickButton(m_joystick, 12);

  public OI() {
    setName("OI -- Driver Inputs");
    setupDashboard();
    m_btn_1.whenPressed(new MoonDriveCommand());
    m_btn_2.whenPressed(new SwitchDashboardView());
    m_btn_3.whenPressed(new StrafeDriveCommand());   
    m_btn_7.whenPressed(new CalibrateSwerveSystemCommand(CalibrationType.kFast));
    m_btn_8.whenPressed(new CalibrateSwerveSystemCommand(CalibrationType.kTight));
    m_btn_10.whenPressed(new CalibrateSwerveUnitCommand(UnitID.kFrontRight, CalibrationType.kTight));
    m_btn_11.whileHeld(new SpinRingCommand(UnitID.kFrontRight, 100.0));
    m_btn_5.whenPressed(new ZeroAngleCommand());
  }

  // Returns the condition of the given button number
  public boolean getRawButton(int id) {
    return m_joystick.getRawButton(id);
  }

  // Returns the main X axis on the Joystick
  public double getX() {
    return m_joystick.getX();
  }

  // Returns the main Y axis on the Joystick (Reverses it)
  public double getY() {
    return -m_joystick.getY();
  }

  // Returns the twist axis on the Joystick
  public double getTwist() {
    return m_joystick.getRawAxis(4);
  }

  // Retruns the trim axis on the Joystick
  public double getTrim() {
    return -m_joystick.getRawAxis(3);
  }

   // Returns condition of all the buttons on the joystick.
   public String getAllButtons() {
    String z = "";
    for(int i = 1; i <= 12; i++) {
      if (m_joystick.getRawButton(i)) {
        z += "T";
      } else {
        z += "F";
      }
    }
    return z;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Inputs");
    builder.addDoubleProperty("X", this::getX, null);
    builder.addDoubleProperty("Y", this::getY, null);
    builder.addDoubleProperty("Twist", this::getTwist, null);
    builder.addDoubleProperty("Trim", this::getTrim, null);
    builder.addStringProperty("Buttons", this::getAllButtons, null);
  }

  // Setups of the Shuffleboard, should be called only once.
  public void setupDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab(RobotMap.MainTabName);
    int ix = RobotMap.SlotZeroForUI.x;
    int iy = RobotMap.SlotZeroForUI.y;
    tab.add(this).withPosition(ix, iy).withSize(2,2);
  }
      
}
