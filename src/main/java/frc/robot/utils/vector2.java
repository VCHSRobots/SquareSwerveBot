/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class vector2 {
    double m_x = 0.0;
    double m_y = 0.0;

    vector2(double x, double y) {
        m_x = x;
        m_y = y;
    }

    vector2() {
        this(0.0, 0.0);
    }

    public double getMagnitude() {
        return Math.sqrt(m_x*m_x + m_y*m_y);
    }

    public double getAngleRadians() {
        return Math.atan2(m_y, m_x);
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }
}
