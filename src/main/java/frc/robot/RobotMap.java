// RobotMap.java -- Provides all mapping of Electrial Devices to Software
// Also provides a place to put other fix constancts such as UI values
// and parameters regarding the robot.  

// MAP For "Square Swerve Bot", built Fall 2019

package frc.robot;

import frc.robot.utils.UIPoint;

public class RobotMap {
  // Spark Controllers for NEO Motors on the Swerve Units
  public static int SU_FL_DriveMotor = 22;  
  public static int SU_FR_DriveMotor = 23;  
  public static int SU_BL_DriveMotor = 21;
  public static int SU_BR_DriveMotor = 24;

  // Talon Controllers for 775 Steering Motors on the Swerve Units
  public static int SU_FL_SteeringMotor = 32;  
  public static int SU_FR_SteeringMotor = 33;  
  public static int SU_BL_SteeringMotor = 31;
  public static int SU_BR_SteeringMotor = 34;

  // Angle Offsets for the Swerve Units, found by experimentation.
  public static double SU_FL_Offset =   40.0;   
  public static double SU_FR_Offset =  -52.0;   
  public static double SU_BL_Offset =  130.0; 
  public static double SU_BR_Offset =  -75.0; 

  // Stuff to set up the UI on ShuffleBoard
  public static String MainTabName = "Square Swerve Bot";
  public static UIPoint SlotZeroForUI = new UIPoint(0,0);  // Location of Basic Inputs
  public static UIPoint SlotZeroForSU = new UIPoint(3,0);  // For layout of each Swerve Unit
  public static UIPoint SlotZeroForSS = new UIPoint(0,3);  // Full Swerve Drive Stuff

  // Stuff that defines the geometery of the bot
  public static double Len_WheelBase = 14.0;   // Distance between wheels, front-back, in inches.
  public static double Len_TrackWidth = 14.0;  // Distance between wheels, left-right, in inches.

  // Stuff that controls how the robot behaves
  public static double MaxSpeed = 15.0;  // In Feet/Second
  public static double MaxSpinRate = 540; // In Degrees/Second
}
