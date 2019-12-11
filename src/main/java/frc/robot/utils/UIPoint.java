// UIPoint.java -- Holds a simple x,y coordinate for UI Widgets

package frc.robot.utils;

// Contains an x,y coordinate that can be accessed many ways.
public class UIPoint {
  public int x;
  public int y;

  public UIPoint(int xx, int yy) {
    x = xx;
    y = yy;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  public void set(int xx, int yy) {
    x = xx;
    y = yy;
  }

  public void setX(int xx) {
    x = xx;
  }

  public void setY(int yy) {
    y = yy;
  }
}
