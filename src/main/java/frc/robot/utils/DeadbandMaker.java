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

  //TODO Fix this, strafe x is backwards. verify signs of quadrants, etc
  /*
  @param v: vector to apply deadband on
  @param r: radius of center circle deadband area, should be larger than l
  @param l: linear bands outside of center cirle deadband to exclude
  */
  public static vector2 addDeadband2D(vector2 v, double r, double l) {
    double x = 0.0;
    double y = 0.0;
    //assert(r >= l);
    if (Math.abs(v.getMagnitude()) > Math.abs(r)) {
      double theta = v.getAngleRadians();

      if (Math.abs(v.getX()) > Math.abs(l)) {
        x = v.getX() - Math.copySign(Math.cos(theta), v.getX());
        x = x / (1 - Math.cos(theta));
      }

      if (Math.abs(v.getY()) > Math.abs(l)) {
        y = v.getY() - Math.copySign(Math.cos(theta), v.getY());
        y = y / (1 - Math.cos(theta));
      }
    }

    return new vector2(x,y);
  }
}