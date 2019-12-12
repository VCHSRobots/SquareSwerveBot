// AngleCals.java -- Does simple calcuations between angles.

package frc.robot.utils;

public class AngleCals {

  // Clamps an arbitary angle into the region -180 to 180 degrees.
  public static double clamp_angle(double angle) {
    angle = angle % 360.0;
     if (angle > 180.0) {
      angle -= 360.0;
    }
    if (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  // Computes the shortest angle to move from angle1 to angle2.
  public static double delta(double angle1, double angle2) {
    double delta = clamp_angle(angle2) - clamp_angle(angle1);
    if (delta < -180.0) {
      delta += 360.0;
    }
    if (delta > 180.0) {
      delta -= 360.0;
    }
    return delta;
  }

  // Computes the average between two angles, 
  public static double average(double angle1, double angle2) {
    double delta = delta(angle1, angle2);
    return clamp_angle(angle1 + 0.5*delta);
  }
}