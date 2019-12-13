// MoonDriveCommand.java -- Move around an object like the moon does the earth.

// In this control system, the algorithm and equations 
// developed by Ether (3/28/2011) found on Chief Delphi
// are implemented for a "moon" motion. This yeilds a 
// control system as follows:
// 
// 1. Move to an object with the standard drive system. Face the
//    robot at the object.
// 2. Initiate moon mode with the trigger button.  Continue
//    to hold the trigger button during the following steps.
// 3. Back away from the object.  During backing, the robot
//    will only go backwards.
// 4. When the robot is in the "orbit" position, then
//    push the control stick forward (past zero) and to one side
//    or the other to circle the object.  The direction of travel
//    is selected by the side the stick leans to.  The speed of
//    travel is controled by displacement from center.
// 5. Release the trigger switch to cancel this command. Its
//    best to be at rest when releasing the trigger, as you will
//    jump back into standard drive mode.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.SwerveUnit;
import frc.robot.subsystems.SwerveDriveSubsystem.UnitID;
import frc.robot.utils.DeadbandMaker;

public class MoonDriveCommand extends Command {
  private double m_r;                    // Calculated once at init = Diagonal from Left-Front to Back-Right
  private double m_lr;                   // Calculated once at init = WheelBase / Diagonal
  private double m_wr;                   // Calculated once at init = TrackWidth / Diagonal
  private double[] m_start0;             // Location of the starting point.
  private int m_istage = 0;              // What stage of this command that we are in.
  private double m_radius;               // The radius of the orbit in inches.
  private boolean m_isFinished = false;  

  public MoonDriveCommand() {
    requires(Robot.m_swerve);
    this.setInterruptible(true);
    m_r = Math.sqrt(RobotMap.Len_TrackWidth*RobotMap.Len_TrackWidth + RobotMap.Len_WheelBase*RobotMap.Len_WheelBase);
    m_lr = RobotMap.Len_WheelBase / m_r;
    m_wr = RobotMap.Len_TrackWidth / m_r;
  }

  @Override
  protected void initialize() {
    super.initialize();
    // Here we "mark" the center of the orbit.  We need to measure how far we back up.
    // This is done by taking an average of the drive encoder values at the starting point,
    // and comparing them to an average when the orbit circle is reached.
    System.out.printf("Starting MoonDriveCommand.\n");
    m_start0 = new double[4];
    m_start0[0] = Robot.m_swerve.getUnit(UnitID.kFrontLeft).getDriveEncoder();
    m_start0[1] = Robot.m_swerve.getUnit(UnitID.kFrontRight).getDriveEncoder();
    m_start0[2] = Robot.m_swerve.getUnit(UnitID.kBackLeft).getDriveEncoder();
    m_start0[3] = Robot.m_swerve.getUnit(UnitID.kBackRight).getDriveEncoder();
    //System.out.printf("m_Starts = %6.1f, %6.1f, %6.1f, %6.1f\n", m_start0[0], m_start0[1], m_start0[2], m_start0[3]);
    m_istage = 0;
    m_isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Automatically finish when trigger button is released.
    if(!Robot.m_oi.getRawButton(1)) {
      m_isFinished = true;
      return;
    }
    switch(m_istage) {
      case 0: stage_0(); return;
      case 1: stage_1(); return;
    }
    m_isFinished = true;
  }

  @Override
  protected boolean isFinished() {
    return m_isFinished; 
  }

  @Override
  protected void end() {
    Robot.m_swerve.stop();
  }

  private void stage_0() {
    // At this stage, the trigger is being held and the bot should
    // be backing up.  Once it stops backing up, swtich stages.
    double fwd = Robot.m_oi.getY();
    fwd = DeadbandMaker.addDeadband(fwd, 0.05);
    if (fwd <= 0.0) {
      fwd = fwd *RobotMap.MaxSpeed;
      Robot.m_swerve.getUnit(UnitID.kFrontRight).setOptimizedSpeedAndDirection(fwd, 0.0);
      Robot.m_swerve.getUnit(UnitID.kFrontLeft).setOptimizedSpeedAndDirection(fwd, 0.0);
      Robot.m_swerve.getUnit(UnitID.kBackLeft).setOptimizedSpeedAndDirection(fwd, 0.0);
      Robot.m_swerve.getUnit(UnitID.kBackRight).setOptimizedSpeedAndDirection(fwd, 0.0);
      return;
    }
    // Okay, the stick has been pushed forward.  Calculate the radius of the orbit.
    double ticks1 = Robot.m_swerve.getUnit(UnitID.kFrontLeft).getDriveEncoder();
    double ticks2 = Robot.m_swerve.getUnit(UnitID.kFrontRight).getDriveEncoder();
    double ticks3 = Robot.m_swerve.getUnit(UnitID.kBackLeft).getDriveEncoder();
    double ticks4 = Robot.m_swerve.getUnit(UnitID.kBackRight).getDriveEncoder();
    //System.out.printf("Ticks at Radius = %6.1f, %6.1f, %6.1f, %6.1f\n", ticks1, ticks2, ticks3, ticks4);
    ticks1 = Math.abs(ticks1 - m_start0[0]);
    ticks2 = Math.abs(ticks2 - m_start0[1]);
    ticks3 = Math.abs(ticks3 - m_start0[2]);
    ticks4 = Math.abs(ticks4 - m_start0[3]);
    double average_ticks = 0.25 * (ticks1 + ticks2 + ticks3 + ticks4);
    m_radius = 0.5*RobotMap.Len_WheelBase + 5 + SwerveUnit.DriveTicksToInches(average_ticks);  
    System.out.printf("m_radius = %10.2f\n", m_radius);
    m_istage = 1;
    //stage_1();  // Optimize here. Save's waiting 50ms before starting stage 1.
  }
  
  private void stage_1() {
    // Here we orbit while facing the center of the orbit.  
    double x = DeadbandMaker.addDeadband(Robot.m_oi.getX(), 0.05);
    double y = DeadbandMaker.addDeadband(Robot.m_oi.getY(), 0.05);
    double speed = Math.sqrt(x*x + y*y) * RobotMap.MaxSpeed * 12;   // Speed of bot along circumference, in inches/sec.
    speed = 0.5 * speed;  // For now, slow things down.
    double circumference = m_radius * 2 * Math.PI;  // Circumference of circle being traveled, in inches.
    double rotations_per_second = speed / circumference;  // (inches/sec) / (inches/rotation) = (rotation/sec) 
    double rcw = 2.0 * Math.PI * rotations_per_second;  // Get radians per sec.
    double str = -(circumference * rotations_per_second) / 12.0;  // should be feet / sec.
    double fwd = 0;
    System.out.printf("radius, speed, rps, rcw, str = %8.1f, %6.1f, %6.1f, %6.1f, %6.1f\n", 
      m_radius, speed, rotations_per_second, rcw, str);


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
    // (Note, this will probably mess up the wheel angles to stay on orbit.)
    if (maxws > RobotMap.MaxSpeed) {
      wsFR = RobotMap.MaxSpeed * wsFR / maxws;
      wsFL = RobotMap.MaxSpeed * wsFL / maxws;
      wsBL = RobotMap.MaxSpeed * wsBL / maxws;
      wsBR = RobotMap.MaxSpeed * wsBR / maxws;
    } 

    //System.out.printf("Angles = %4.1f, %4.1f, %4.1f, %4.1f\n", waFL, waFR, waBL, waBR);
    //System.out.printf("Speeds = %4.1f, %4.1f, %4.1f, %4.1f\n\n", wsFL, wsFR, wsBL, wsBR);
    // Optimize here.  For each wheel, based on their current position
    // and direction, it might be better to reverse the angle and direction.
    // That is, why turn the steering ring 180 degrees when we can just 
    // reverse the drive?  Also, if we are not moving, why spin the steering rings?

    Robot.m_swerve.getUnit(UnitID.kFrontRight).setOptimizedSpeedAndDirection(wsFR, waFR);
    Robot.m_swerve.getUnit(UnitID.kFrontLeft).setOptimizedSpeedAndDirection(wsFL, waFL);
    Robot.m_swerve.getUnit(UnitID.kBackLeft).setOptimizedSpeedAndDirection(wsBL, waBL);
    Robot.m_swerve.getUnit(UnitID.kBackRight).setOptimizedSpeedAndDirection(wsBR, waBR);
  
  }

}