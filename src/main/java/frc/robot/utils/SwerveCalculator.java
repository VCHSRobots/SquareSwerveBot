// SwerveCalculator.java -- object to calculate swerve angles.

package frc.robot.utils;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;

// Contains an x,y coordinate that can be accessed many ways.
public class SwerveCalculator {
  private double m_r;                    // Calculated once at init = Diagonal from Left-Front to Back-Right
  private double m_lr;                   // Calculated once at init = WheelBase / Diagonal
  private double m_wr;                   // Calculated once at init = TrackWidth / Diagonal
  private double m_gyro_angle = 0.0;     // For future features.  Just set this in the execute() routine.
  private double m_spin_scale = 1.0;     // Calculated once at init 
  // Calculated Outputs
  private double m_waFR;
  private double m_waFL;
  private double m_waBL;
  private double m_waBR;
  private double m_wsFR;
  private double m_wsFL;
  private double m_wsBL;
  private double m_wsBR;
  
  public SwerveCalculator(double wheelBase, double trackWidth, double maxSpeed, double maxSpinRate) {
    m_r = Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase);
    m_lr = wheelBase / m_r;
    m_wr = trackWidth / m_r;
    double max_unchecked_spin = 360.0 * maxSpeed * 12.0 / (Math.PI * m_r);  // (degrees) (feet/sec) (inchs/foot) / (inches) = (degrees/sec)
    m_spin_scale = maxSpinRate / max_unchecked_spin;
  }

  // Sets the inputs to the calculator. The inputs are fwd for forward, str for 
  // strafing to the right, rcw for rotating clockwize, and the gyro_angle, in
  // degrees.  The fwd, str, and rcw are normalized for the range -1 to 1.
  // If not using field-centric orienation, use zero for gyro_angele.
  public void setInputs(double fwd, double str, double rcw, double gyro_angle) {

    // Here, we need to scale back the twist motion.  If unchecked, a full twist of 1.0
    // would lead to full speed on the wheels -- which for a normal robot, would be
    // on the order of 1000 degrees per second.
    rcw = rcw * m_spin_scale;

    // Convert the inputs for use in field-centric orienation.  If 
    // field-centric orientation is not used, set the gyro angle to zero.
    double g = m_gyro_angle * Math.PI / 180.0;
    double temp = fwd * Math.cos(g) + str * Math.sin(g);
    str = -fwd * Math.sin(g) + str * Math.cos(g);
    fwd = temp;

    double a = str - rcw * m_lr;
    double b = str + rcw * m_lr;
    double c = fwd - rcw * m_wr;
    double d = fwd + rcw * m_wr;

    // Wheel Speeds...
    m_wsFR = Math.sqrt(b*b + c*c); 
    m_wsFL = Math.sqrt(b*b + d*d);
    m_wsBL = Math.sqrt(a*a + d*d);
    m_wsBR = Math.sqrt(a*a + c*c);

    // Wheel Angles...
    m_waFR = Math.atan2(b, c) * 180.0 / Math.PI;
    m_waFL = Math.atan2(b, d) * 180.0 / Math.PI;
    m_waBL = Math.atan2(a, d) * 180.0 / Math.PI;
    m_waBR = Math.atan2(a, c) * 180.0 / Math.PI;

    // Max Wheel Speed
    double maxws = Math.max(m_wsFL, m_wsFR);
    maxws = Math.max(maxws, m_wsBL);
    maxws = Math.max(maxws, m_wsBR);

    // Don't allow any wheels to go over the max.  If they
    // do, then scale back all by the same factor.
    if (maxws > 1.0) {
      m_wsFR = m_wsFR / maxws;
      m_wsFL = m_wsFL / maxws;
      m_wsBL = m_wsBL / maxws;
      m_wsBR = m_wsBR / maxws;
    }

    m_wsFR = m_wsFR * RobotMap.MaxSpeed;  
    m_wsFL = m_wsFL * RobotMap.MaxSpeed;  
    m_wsBL = m_wsBL * RobotMap.MaxSpeed;  
    m_wsBR = m_wsBR * RobotMap.MaxSpeed;  
  }

  // Returns the calculated speeds, in an array, ordered as FL, FR, BL, BR.
  public double[] getSpeeds() {
    double x[] = new double[4];
    x[0] = m_wsFL;
    x[1] = m_wsFR;
    x[2] = m_wsBL;
    x[3] = m_wsBR;
    return x;
  }

  // Returns the calculated angles, in an array, ordered as FL, FR, BL, BR.
  public double[] getAngles() {
    double x[] = new double[4];
    x[0] = m_waFL;
    x[1] = m_waFR;
    x[2] = m_waBL;
    x[3] = m_waBR;
    return x;
  }

  // Sends the most recently computed outputs to the swerve drive system.
  public void sendOutputsToSwerveSystem() {

    // Optimize here.  For each wheel, based on their current position
    // and direction, it might be better to reverse the angle and direction.
    // That is, why turn the steering ring 180 degrees when we can just 
    // reverse the drive?  Also, if we are not moving, why spin the steering rings?

    Robot.m_swerve.getUnit(UnitID.kFrontLeft).setOptimizedSpeedAndDirection(m_wsFL, m_waFL);
    Robot.m_swerve.getUnit(UnitID.kFrontRight).setOptimizedSpeedAndDirection(m_wsFR, m_waFR);
    Robot.m_swerve.getUnit(UnitID.kBackLeft).setOptimizedSpeedAndDirection(m_wsBL, m_waBL);
    Robot.m_swerve.getUnit(UnitID.kBackRight).setOptimizedSpeedAndDirection(m_wsBR, m_waBR);
  }
}
