// SparkMaxConfiguration.java -- holds a configuration for a SPARK Max Controller

package frc.robot.utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;

/**
 * An example command.  You can replace me with your own command.
 */
public class SparkMaxConfiguration {
  public double m_P = 0.0;
  public double m_I = 0.0;
  public double m_D = 0.0;
  public double m_F = 0.0;
  public double m_IZone = 0.0;
  public double m_MaxOutput = 1.0;
  public double m_MinOutput = -1.0;

  public SparkMaxConfiguration() {
  }

  public SparkMaxConfiguration(double p, double i, double d, double f) {
    m_P = p;
    m_I = i;
    m_D = d;
    m_F = f;
  }

  public double getP() { return m_P; }
  public double getI() { return m_I; }
  public double getD() { return m_D; }
  public double getF() { return m_F; }
  public double getIZone() { return m_IZone; }
  public double getMaxOutput() { return m_MaxOutput; }
  public double getMinOutput() { return m_MinOutput; }

  public void setP(double x) { m_P = x; }
  public void setI(double x) { m_I = x; }
  public void setD(double x) { m_D = x; }
  public void setF(double x) { m_F = x; }
  public void setIZone(double x) { m_IZone = x; }
  public void setMaxOutput(double x) { m_MaxOutput = x; }
  public void setMinOutput(double x) { m_MinOutput = x; }

  public void configureController(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();
    pid.setP(m_P);
    pid.setI(m_I);
    pid.setD(m_D);
    pid.setFF(m_F);
    pid.setIZone(m_IZone);
    pid.setOutputRange(m_MinOutput, m_MaxOutput);
  }
}

  

  