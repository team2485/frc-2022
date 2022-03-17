// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class Turret extends SubsystemBase implements Loggable {
  private final WPI_TalonSRX m_talon = new WPI_TalonSRX(kTurretTalonPort);
  private final AnalogPotentiometer m_potentiometer =
      new AnalogPotentiometer(
          kTurretPotentiometerChannel,
          kTurretPotentiometerRangeOfMotion,
          kTurretPotentiometerOffset);

  private final DigitalInput m_ccwSlotSensor = new DigitalInput(kTurretCCWSlotSensorPort);
  private final DigitalInput m_cwSlotSensor = new DigitalInput(kTurretCWSlotSensorPort);

  private final SR_ProfiledPIDController m_pidController =
      new SR_ProfiledPIDController(
          kPTurretVoltsPerRadian, 0, kDTurretVoltSecondsPerRadian, kTurretMotionProfileConstraints);

  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
          kSTurretVolts, kVTurretVoltSecondsPerRadian, kATurretVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  private double m_angleSetpointRadians = 0;

  private double m_lastVelocitySetpoint = 0;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  private double m_lastAngleRadians = 0;
  private double m_velocityRadiansPerSecond = 0;

  @Log(name = "Feedback output")
  private double m_feedbackOutput = 0;

  @Log(name = "Feedforward output")
  private double m_feedforwardOutput = 0;

  @Log(name = "Output voltage")
  private double m_lastOutputVoltage = 0;

  @Log(name = "filtered angle")
  private double m_filteredPotentiometerAngle;

  @Log(name = "filtered velocity")
  private double m_filteredVelocity = 0;

  private boolean m_enabled = true;

  public Turret() {

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.peakCurrentLimit = kTurretSupplyCurrentThresholdAmps;
    talonConfig.peakCurrentDuration = kTurretSupplyCurrentThresholdTimeMs;
    talonConfig.continuousCurrentLimit = kTurretSupplyCurrentLimitAmps;

    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
    m_talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);

    m_talon.setInverted(false);

    m_filteredPotentiometerAngle = this.getUnfilteredAngleRadians();
    m_angleSetpointRadians = this.getUnfilteredAngleRadians();

    Shuffleboard.getTab("Turret").add("Turret controller", m_pidController);
    Shuffleboard.getTab("Hood").add("Turret feedforward", m_feedforward);
  }

  // ccw + cw-, 0 straight forward
  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadians =
        MathUtil.clamp(angle % (2 * Math.PI), kTurretMinPositionRadians, kTurretMaxPositionRadians);
  }

  @Log(name = "Current unfiltered angle (radians)")
  public double getUnfilteredAngleRadians() {
    return -m_potentiometer.get() % (2 * Math.PI);
  }

  public double getFilteredAngleRadians() {
    return m_filteredPotentiometerAngle;
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(this.getUnfilteredAngleRadians());
  }

  @Log(name = "Velocity (radps)")
  public double getVelocityRadiansPerSecond() {
    return m_velocityRadiansPerSecond;
  }

  @Log(name = "At goal")
  public boolean atGoal() {
    return Math.abs(m_filteredPotentiometerAngle - m_angleSetpointRadians)
        < kTurretPositionTolerance;
  }

  @Log(name = "Counter-clockwise slot sensor")
  private boolean getCCWSlotSensor() {
    return m_ccwSlotSensor.get();
  }

  @Log(name = "Clockwise slot sensor")
  private boolean getCWLimitSwitch() {
    return m_cwSlotSensor.get();
  }

  public void enable(boolean enabled) {
    m_enabled = enabled;
    if (enabled) {
      m_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
      m_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
      m_talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    } else {
      m_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
      m_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
      m_talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    }
  }

  @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    double outputVoltage = 0;
    if (m_enabled && DriverStation.isEnabled()) {
      double currentAngleRadians = this.getFilteredAngleRadians();
      m_velocityRadiansPerSecond =
          (currentAngleRadians - m_lastAngleRadians) / kTurretLoopTimeSeconds;

      m_filteredVelocity = (m_velocityRadiansPerSecond - m_filteredVelocity) * 0.05;
      m_lastAngleRadians = currentAngleRadians;

      if (m_voltageOverride) {
        outputVoltage = m_voltageSetpoint;
      } else {
        double feedbackOutputVoltage =
            m_pidController.calculate(currentAngleRadians, m_angleSetpointRadians);

        double feedforwardOutputVoltage =
            m_feedforward.calculate(
                m_lastVelocitySetpoint,
                m_pidController.getSetpoint().velocity,
                kTurretLoopTimeSeconds);

        m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;

        m_feedbackOutput = feedbackOutputVoltage;
        m_feedforwardOutput = feedforwardOutputVoltage;

        if (!this.atGoal()) {
          outputVoltage = feedbackOutputVoltage + feedforwardOutputVoltage;
        } else {
          m_talon.set(ControlMode.PercentOutput, 0);
        }
      }
    }

    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.set(ControlMode.PercentOutput, outputVoltage / Constants.kNominalVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  public void periodic() {
    this.runControlLoop();
    m_filteredPotentiometerAngle +=
        (this.getUnfilteredAngleRadians() - m_filteredPotentiometerAngle) * 0.5;
  }
}
