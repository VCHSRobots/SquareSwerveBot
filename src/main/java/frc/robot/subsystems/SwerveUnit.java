// SwerveUnit.java -- Driver for a single Swerve Unit

// Basic Facts regarding the Square Drive Prototype Made in Fall 2019:
//
// Drive Base:  The Wheels are arranged in a square configuration, 
// where the center of the wheels where they touch the ground is 14 inches.
// The wheels are 4 inches in diameter.
//
// A Neo motor, connected to a Spark-Max controller is used for driving.
// The gears between the wheel and the motor are 12:36 then 18:36, 
// giving a 1:6 ratio.
//
// The Steering is done with a 775 motor and a versaplanetary gear box
// that interfaces with a 132 Tooth ring through a 36T spur gear.
// The versa-planetery gear box is configured for 50:1. Therefore,
// The total gear reduction from the 775 to the steering ring is
// (132/36)*50 = 1:183.333.  
//
// An encoder is placed on the output stage of the versaplanentary
// gear box.  Therfore, there are (132/36) rotations on the
// encoder for every one rotation of the steering ring. The encoder
// produces 4096 ticks per rotation, so it produces 15018.66666
// ticks per one rotation of the steering ring.
//
// There is a Hall Effect Sensor located on the diagonal of
// the unit.  This sensor reads "true" for one position of
// the steering ring, and false for all other positions. Therefore
// this sensor can be used to move the steering ring to a
// known position.  However, because each unit is installed
// with a different orientation in repect to the robot, each
// offset from the known position is different for each unit.
//
// The Hall Effect sensor is connected to the drive motor -- not
// the steering motor, which would be more intuative.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.utils.AngleCals;
import frc.robot.utils.SparkMaxConfiguration;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Class to manage/drive one Swerve Unit.  
public class SwerveUnit extends SendableBase {
    private static double m_ticks_per_ring_rotation = 4096.0 * 132.0 / 36.0;
    // Circumference inches per motor rotation = pi*diameter / 6 
    private static double m_inchs_per_rotaions = 3.141592654 * 4.0 / 6.0; 
    private double m_epsilon = 0.0000001;  // Smallest value before changing PID coefficents.
    private double m_offset = 0.0;
    private boolean m_enabled = true;
    private boolean m_isCalibrated = false;
    private String m_calibrationType = "None";
    private double m_desired_angle = 0.0;
    private double m_desired_rpm = 0.0;
    private double m_desired_steering_ticks = 0.0;
    private NetworkTableEntry m_ntentry_enable;
    private NetworkTableEntry m_ntentry_cal;
    private CANSparkMax m_driveMotor;
    private WPI_TalonSRX m_steeringMotor;
    private CANDigitalInput m_limitswitch;
    private String m_name = "?";
    private int m_islot;
    private int m_steeringmode = 0;  // 0=percentate, 1=RPM, 2=Position
    private int m_drivemode = 0;     // 0=percentate, 1=RPM, 2=Position
    private SlotConfiguration m_PID_Steering_RPM;
    private SlotConfiguration m_PID_Steering_POS;
    private SparkMaxConfiguration m_PID_Drive_RPM;
    private int m_getConfigCount = 0;

    public SwerveUnit(String name, int slot, double offset, int drive_id, int steering_id) {
        m_name = name;
        m_islot = slot;
        m_offset = offset;

        m_driveMotor = new CANSparkMax(drive_id, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setOpenLoopRampRate(0.15);
        m_driveMotor.setClosedLoopRampRate(0.05);
        m_driveMotor.setSmartCurrentLimit(20, 10);
        m_driveMotor.setSecondaryCurrentLimit(20);
        m_driveMotor.enableVoltageCompensation(10.5);

        // og  p 5e-5  i 1e-6  d 0.0  f 0.0  izone -/+0.0
        m_PID_Drive_RPM = new SparkMaxConfiguration(3e-6, 3e-7, 0.0, 2e-4, 600);
        m_PID_Drive_RPM.configureController(m_driveMotor);
        //m_driveMotor.getPIDController().set

        m_drivemode = 2;

        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_driveMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_driveMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

        m_limitswitch = new CANDigitalInput(m_driveMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
        m_limitswitch.enableLimitSwitch(false);

        m_steeringMotor = new WPI_TalonSRX(steering_id);
        m_steeringMotor.configFactoryDefault();
        m_steeringMotor.setNeutralMode(NeutralMode.Brake);
        m_steeringMotor.enableCurrentLimit(true);
        m_steeringMotor.configPeakCurrentLimit(25);
        m_steeringMotor.configContinuousCurrentLimit(20);
        m_steeringMotor.configForwardSoftLimitEnable(false);
        m_steeringMotor.configReverseSoftLimitEnable(false);
        m_steeringMotor.setInverted(false);
        
        // The motors are capable of over 360 degrees/sec with constansts below.
        m_steeringMotor.config_kF(0, 0.5);  // Produces about 220 degrees/sec
        m_steeringMotor.config_kP(0, 1.0);  // Set P constant.
        m_steeringMotor.config_kI(0, 0.0);
        m_steeringMotor.config_kD(0, 0.0);

        // When the steering motor is used in seek mode
        m_steeringMotor.config_kF(1, 0.0);  
        m_steeringMotor.config_kP(1, 0.33);  
        m_steeringMotor.config_kI(1, 0.0);
        m_steeringMotor.config_kD(1, 3.0);
        m_steeringMotor.config_IntegralZone(1, 250);

        m_steeringMotor.configPeakOutputForward(0.75);  
        m_steeringMotor.configPeakOutputReverse(-0.75);
        m_steeringMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_2Ms);
        m_steeringMotor.selectProfileSlot(0, 0);
        renew_config_status();
        setName("SU_" + name);
        setupDashboard();
        report();
    }

    // Sets the Drive Motor Output, as a percentage.
    public void setDrivePercentage(double p) {
      if (!m_enabled) return;
      m_driveMotor.set(p);
    }

    // Set Drive speed in units of feet/sec.  A PID controller
    // is used to maintain this speed.
    public void setDriveSpeed(double rate) {
      if (!m_enabled) return;
      double rpm = 60.0 * 6 * (rate * 12) / (3.141592654 * 4);
      if (m_drivemode != 2) {
        m_PID_Drive_RPM.configureController(m_driveMotor);
        m_drivemode = 2;
      }
      m_desired_rpm = rpm;
      m_driveMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    // Sets the Steering Motor Output, as a percentage.
    public void setSteeringPercentage(double p) {
      if (!m_enabled) return;
      m_steeringMotor.set(ControlMode.PercentOutput, p);
      m_steeringmode = 0;
    }

    // Sets the movement of steering ring to the given
    // rate, in degrees per second.  A PID controller
    // in the TALON is used to achieve and maintain this rate.
    public void setSteeringRate(double rate) {
      if (!m_enabled) return;
      // Calculate ticks per 100ms for the given rate.
      double ticks_per_second = m_ticks_per_ring_rotation * rate / 360.0;
      double ticks_per_100ms = ticks_per_second / 10.0;
      if (m_steeringmode != 1) {
        m_steeringMotor.selectProfileSlot(0, 0);
        m_steeringmode = 1;
      }
      m_steeringMotor.set(ControlMode.Velocity, ticks_per_100ms);
    }

     // Uses a PID controller to move the ring to the given angle.
    public void seekToAngle(double angle) {
      if (!m_enabled) return;
      m_desired_angle = AngleCals.clamp_angle(angle);
      double current_angle = getSteeringAngle();
      double delta = AngleCals.delta(current_angle, m_desired_angle);
      m_desired_steering_ticks = getSteeringEncoder() + (delta * m_ticks_per_ring_rotation / 360.0);
      if (m_steeringmode != 2) {
        m_steeringMotor.selectProfileSlot(1, 0);
        m_steeringmode = 2;
      }
      m_steeringMotor.set(ControlMode.Position, m_desired_steering_ticks);
    }

    // Optimized the input settings to reduce moving the steering ring.  If we
    // can move the ring in the opposite direction and reverse the speed, do that.
    public void setOptimizedSpeedAndDirection(double speed, double angle) {
      double delta = Math.abs(AngleCals.delta(getSteeringAngle(), angle));
      if (delta > 90.0) {
        angle = angle - 180.0;
        speed = -speed;
      }
      setDriveSpeed(speed);
      // Also, if the speed is zero, don't bother to move the steering ring.
      if (Math.abs(speed) > 0.03) { 
        seekToAngle(angle);
      }
    }

    // Stop() -- Stops all motors on the swerve unit.
    public void stop() {
      m_driveMotor.set(0.0);
      m_steeringMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // Return Steering Angle, in degrees, from -180 to 180.  When
    // Calibrated, this is relative to the front of the robot.
    public double getSteeringAngle() {
        double angle = getAbsoluteAngle();
        angle = AngleCals.clamp_angle(angle);
        return angle;
    }

    // Returns Absolute Angle, i.e., no adjustments for wrap arounds.
    public double getAbsoluteAngle() {
      double angle = (360.0*getSteeringEncoder() / m_ticks_per_ring_rotation) - m_offset;
      return angle;
    }

    // Return native rotations of the drive motor.
    public double getDriveEncoder() {
        return m_driveMotor.getEncoder().getPosition();
    }

    // Converts measured ticks on the drive unit to inches. (Note that on a NEO
    // motor, one tick is one revolution of the motor shaft.)
    public static double DriveTicksToInches(double nTicks) {
      double dist = nTicks * m_inchs_per_rotaions;
      return dist;
    }

    // Return Raw Ticks of Steering Encoder
    public double getSteeringEncoder() {
        return m_steeringMotor.getSelectedSensorPosition();
    }

    // The rate, in degrees/sec, that the ring is moving.
    public double getSteeringRate() {
        double ticks_per_100ms = m_steeringMotor.getSelectedSensorVelocity();
        double ticks_per_second = 10 * ticks_per_100ms;
        double d = 360.0 * ticks_per_second / m_ticks_per_ring_rotation;
        return d;
    }

    // Returns the Percentage output beting feed to the Steering Motor.
    public double getSteeringOutput() {
      return m_steeringMotor.getMotorOutputPercent();
    }

    // Returns the Angle Offset for this Swerve Unit
    public double getOffset() {
        return m_offset;
    }

    // Returns the calculated speed, in feet/sec, of this drive unit.
    public double getSpeed() {
        double s = m_driveMotor.getEncoder().getVelocity();  // Velocity is RPM at motor.
        s = 3.141592654 * 4.0 * s / (6.0*12.0*60.0);  // Take into account gear ratio and wheel size.
        return s;
    }

    // Returns True if the swerve unit steering ring is at it's prime position.
    public boolean isAtPrime() {
        return m_limitswitch.get();
    }

    // Returns the drive current, in Amps.
    public double getDriveCurrent() {
        return m_driveMotor.getOutputCurrent();
    }

    // Returns the steering current, in Amps.
    public double getSteeringCurrent() {
        return m_steeringMotor.getOutputCurrent();
    }

    // Returns True if this unit is calibrated.
    public boolean isCalibrated() {
        return m_isCalibrated;
    }

    // Returns True if this unit is tightly calibrated.
    public boolean isTightCalibrated() {
      if (m_isCalibrated && m_calibrationType == "Tight") return true;
      return false;
    }

    // Resets the steering encoder.  Unless this is
    // done when the steering ring is at it's prime 
    // possition, this will render the unit uncalibrated.
    // Therefore, this function also resets the calibration 
    // status.  It is up to the caller to set things right.
    public void reset_steering_encoder() {
      m_steeringMotor.setSelectedSensorPosition(0);
      resetCalibration();
    }

    // Sets the calibrated status of this unit.
    public void setCalibrated(String caltype) {
        m_isCalibrated = true;
        m_calibrationType = caltype;
    }

    // Resets the calibration status of this unit, to "uncalibrated".
    public void resetCalibration() {
        m_isCalibrated = false;
        m_calibrationType = "None";       
    }

    private void renew_config_status() {
      TalonSRXConfiguration x = new TalonSRXConfiguration();
      m_steeringMotor.getAllConfigs(x);
      m_PID_Steering_POS = x.slot1;
      m_PID_Steering_RPM = x.slot0;
      m_getConfigCount += 1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveUnit");
        builder.addDoubleProperty("_NEO Velocity", () -> m_driveMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("_NEO Velocity Target", () -> m_desired_rpm, null);
        builder.addDoubleProperty ("Steering Angle", this::getSteeringAngle, null);
        builder.addDoubleProperty ("Desired Angle", () -> m_desired_angle, null);
        builder.addDoubleProperty ("Steering Encoder", this::getSteeringEncoder, null);
        builder.addDoubleProperty ("Steering Target", () -> m_desired_steering_ticks, null);
        builder.addDoubleProperty ("Drive Encoder", this::getDriveEncoder, null);
        builder.addDoubleProperty ("_Speed", this::getSpeed, null);
        builder.addDoubleProperty ("Drive Current", this::getDriveCurrent, null);
        builder.addDoubleProperty ("Steering Current", this::getSteeringCurrent, null);
        builder.addDoubleProperty ("Steering Rate", this::getSteeringRate, null);
        builder.addDoubleProperty ("Steering Output", this::getSteeringOutput, null);
        builder.addBooleanProperty("At Prime", this::isAtPrime, null);
        builder.addDoubleProperty ("Offset", this::getOffset, null);
        builder.addBooleanProperty("Calibrated", this::isCalibrated, null);
        builder.addStringProperty ("CalType", () -> m_calibrationType, null);
        builder.addBooleanProperty("Enabled", () -> m_enabled, null);
        builder.addDoubleProperty("PID_SRPM_P", () -> m_PID_Steering_RPM.kP, null);
        builder.addDoubleProperty("PID_SRPM_I", () -> m_PID_Steering_RPM.kI, null);
        builder.addDoubleProperty("PID_SRPM_D", () -> m_PID_Steering_RPM.kD, null);
        builder.addDoubleProperty("PID_SRPM_F", () -> m_PID_Steering_RPM.kF, null);
        builder.addDoubleProperty("PID_SPOS_P", () -> m_PID_Steering_POS.kP, null);
        builder.addDoubleProperty("PID_SPOS_I", () -> m_PID_Steering_POS.kI, null);
        builder.addDoubleProperty("PID_SPOS_D", () -> m_PID_Steering_POS.kD, null);
        builder.addDoubleProperty("PID_SPOS_F", () -> m_PID_Steering_POS.kF, null);
        builder.addDoubleProperty("UpdateCnt", () -> m_getConfigCount, null);
    }

    // Setup the dashboard
    public void setupDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(RobotMap.MainTabName);
        m_ntentry_enable = tab.add(m_name + "_Enable", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withPosition(m_islot + 1, 0).withSize(1,1).getEntry();
        m_ntentry_cal  = tab.add(m_name + "_Calibrated", false)
           .withWidget(BuiltInWidgets.kBooleanBox)
           .withPosition(m_islot, 0).withSize(1,1).getEntry();
        tab.add(this).withPosition(m_islot, 1).withSize(2,6);
    }

    public void report() {
        m_ntentry_cal.setBoolean(isCalibrated());
    }

    public void readFromDashboard() {
      m_enabled = m_ntentry_enable.getBoolean(true);
      if (!m_enabled) {
        // Shut off motors if dashboard says to disable.
        stop();
      }
    }
}
