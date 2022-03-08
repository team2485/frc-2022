// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
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

  @Log(name = "filtered angle")
  private double m_filteredPotentiometerAngle;

  public Turret() {

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.peakCurrentLimit = kTurretSupplyCurrentThresholdAmps;
    talonConfig.peakCurrentDuration = kTurretSupplyCurrentThresholdTimeMs;
    talonConfig.continuousCurrentLimit = kTurretSupplyCurrentLimitAmps;

    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setStatusFramePeriod(1, 255);
    m_talon.setStatusFramePeriod(2, 255);

    m_filteredPotentiometerAngle = this.getAngleRadians();

    Shuffleboard.getTab("Turret").add("Turret controller", m_pidController);
    Shuffleboard.getTab("Hood").add("Turret feedforward", m_feedforward);
  }

  @Config(name = "Set angle (radians)")
  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadians =
        MathUtil.clamp(angle % (2 * Math.PI), kTurretMinPositionRadians, kTurretMaxPositionRadians);
  }

  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_potentiometer.get() % (2 * Math.PI);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(this.getAngleRadians());
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

  @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    double currentAngleRadians = m_filteredPotentiometerAngle;
    m_velocityRadiansPerSecond =
        (currentAngleRadians - m_lastAngleRadians) / kTurretLoopTimeSeconds;

    m_lastAngleRadians = currentAngleRadians;

    if (m_voltageOverride) {
      m_talon.set(ControlMode.PercentOutput, m_voltageSetpoint / Constants.kNominalVoltage);
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
        m_talon.set(
            ControlMode.PercentOutput,
            (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage);
      }
    }
  }

  public void periodic() {
    this.runControlLoop();
    m_filteredPotentiometerAngle += (this.getAngleRadians() - m_filteredPotentiometerAngle) * 0.5;
  }
}
