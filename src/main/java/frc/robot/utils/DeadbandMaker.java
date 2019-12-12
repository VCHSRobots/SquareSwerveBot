// DeadbandMaker.java -- Calculations for deadbands

package frc.robot.utils;


public class DeadbandMaker {

  // Adds a deadband centered at zero, and scaled for a -1 to 1 control
  // region.  The size of the deadband is given by "p" which is the
  // percent of the region from -1 to 1 that should be dead.  For example a p of
  // 0.1 means that 10% of the full region, centered at zero will be dead.
  public static double addDeadband(double v, double p) {
    double p2 = 0.5 * p;
    if (v < p2 && v > -p2) {
      return 0.0;
    }
    if (v < 0.0) {
      v = v + p2;
    } else {
      v = v - p2;
    }
    return v / (1 - p2);
  }
}