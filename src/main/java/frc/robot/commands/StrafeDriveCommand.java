// StrafeDriveCommand.java -- Strafe Drive Mode

// This mode under development.  Currently, by holding down button 3, causes 
// the twist to be ignored, and will drive streight.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;
import frc.robot.utils.DeadbandMaker;

// Simple Joystick connection to Swerve Drive.
public class StrafeDriveCommand extends Command {
  private double m_r;                    // Calculated once at init = Diagonal from Left-Front to Back-Right
  private double m_lr;                   // Calculated once at init = WheelBase / Diagonal
  private double m_wr;                   // Calculated once at init = TrackWidth / Diagonal
  private double m_gyro_angle = 0.0;     // For future features.  Just set this in the execute() routine.
  private double m_spin_scale = 1.0;     // Calculated once at init 
  private boolean m_isFinished = false;

  public StrafeDriveCommand() {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
  }

  @Override
  protected void initialize() {
    super.initialize();
    m_r = Math.sqrt(RobotMap.Len_TrackWidth*RobotMap.Len_TrackWidth + RobotMap.Len_WheelBase*RobotMap.Len_WheelBase);
    m_lr = RobotMap.Len_WheelBase / m_r;
    m_wr = RobotMap.Len_TrackWidth / m_r;
    double max_unchecked_spin = 360.0 * RobotMap.MaxSpeed * 12.0 / (Math.PI * m_r);
    m_spin_scale = RobotMap.MaxSpinRate / max_unchecked_spin;
    m_isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!Robot.m_oi.getRawButton(3)) {
      m_isFinished = true;
      return;
    } 
    double str = Robot.m_oi.getX();
    double fwd = Robot.m_oi.getY();
    double rcw = 0;

    // The stick is sensitive.  HEre we introduce a deadband on each axis.
    str = DeadbandMaker.addDeadband(str, 0.5);
    fwd = DeadbandMaker.addDeadband(fwd, 0.5);
    //rcw = 0; assigned above

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
    double wsFR = Math.sqrt(b*b + c*c); 
    double wsFL = Math.sqrt(b*b + d*d);
    double wsBL = Math.sqrt(a*a + d*d);
    double wsBR = Math.sqrt(a*a + c*c);

    // Wheel Angles...
    double waFR = Math.atan2(b, c) * 180.0 / Math.PI;
    double waFL = Math.atan2(b, d) * 180.0 / Math.PI;
    double waBL = Math.atan2(a, d) * 180.0 / Math.PI;
    double waBR = Math.atan2(a, c) * 180.0 / Math.PI;

    // Max Wheel Speed
    double maxws = Math.max(wsFL, wsFR);
    maxws = Math.max(maxws, wsBL);
    maxws = Math.max(maxws, wsBR);

    // Don't allow any wheels to go over the max.  If they
    // do, then scale back all by the same factor.
    if (maxws > 1.0) {
      wsFR = wsFR / maxws;
      wsFL = wsFL / maxws;
      wsBL = wsBL / maxws;
      wsBR = wsBR / maxws;
    }

    wsFR = wsFR * RobotMap.MaxSpeed;  
    wsFL = wsFL * RobotMap.MaxSpeed;  
    wsBL = wsBL * RobotMap.MaxSpeed;  
    wsBR = wsBR * RobotMap.MaxSpeed;  

    // Optimize here.  For each wheel, based on their current position
    // and direction, it might be better to reverse the angle and direction.
    // That is, why turn the steering ring 180 degrees when we can just 
    // reverse the drive?  Also, if we are not moving, why spin the steering rings?

    Robot.m_swerve.getUnit(UnitID.kFrontRight).setOptimizedSpeedAndDirection(wsFR, waFR);
    Robot.m_swerve.getUnit(UnitID.kFrontLeft).setOptimizedSpeedAndDirection(wsFL, waFL);
    Robot.m_swerve.getUnit(UnitID.kBackLeft).setOptimizedSpeedAndDirection(wsBL, waBL);
    Robot.m_swerve.getUnit(UnitID.kBackRight).setOptimizedSpeedAndDirection(wsBR, waBR);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_isFinished; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_swerve.stop();
  }
}