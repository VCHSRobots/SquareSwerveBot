// GyroObject.java -- Simple Class to Display Measured Angle in a Gyro Widget

package frc.robot.utils;

import edu.wpi.first.wpilibj.GyroBase;


// Class to aid in display of angle of swerve drive unit. 
public class GyroObject extends GyroBase {
    private double m_angle = 0;

    public GyroObject() {
        m_angle = 0;
    }

    public void setMeasuredAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) {
            angle += 360.0;
        }
        m_angle = angle;
    }

    @Override
    public double getAngle() {
        return m_angle;
    }

    @Override
    public double getRate() {
        return 0.0;
    }

    @Override
    public void reset() {
    }

    @Override
    public void calibrate() {
    }

}