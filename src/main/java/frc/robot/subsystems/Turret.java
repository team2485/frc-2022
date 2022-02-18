// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.math.BufferZone;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.annotations.*;

public class Turret extends SubsystemBase {
  private final WPI_TalonSRX m_talon = new WPI_TalonSRX(kTurretTalonPort);
  private final AnalogPotentiometer m_encoder =
      new AnalogPotentiometer(kTurretEncoderChannel, kTurretRangeOfMotion, kTurretOffset);

  private ProfiledPIDController m_controller =
      new ProfiledPIDController(
          kP,
          0,
          kD,
          new TrapezoidProfile.Constraints(
              kMaxVelocityRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared));

  private BufferZone m_bufferZone =
      new BufferZone(
          -kMaxVelocityRadiansPerSecond,
          -kMaxVelocityRadiansPerSecond,
          kMinPositionRadians,
          kMaxPositionRadians,
          kBufferSizeRadians);

  @Log(name = "Turret Feedforward")
  private SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(kSVolts, kVVoltSecondsPerMeter, kAVoltSecondsSquaredPerMeter);

  // Angle coord system is 0 forward, - CW, + CCW
  private double m_angleSetpointRadians = 0;
  private double m_previousVelocitySetpoint = 0;

  public Turret() {
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.peakCurrentLimit = kTurretSupplyCurrentThresholdAmps;
    talonConfig.peakCurrentDuration = kTurretSupplyCurrentThresholdTimeMs;
    talonConfig.continuousCurrentLimit = kTurretSupplyCurrentLimitAmps;

    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);

    m_talon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

  @Config(name = "Set angle (radians)")
  public void setAngleRadians(double angle) {
    m_angleSetpointRadians = MathUtil.clamp(angle, kMinPositionRadians, kMaxPositionRadians);
  }

  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_encoder.get();
  }

  @Log(name = "Counter-clockwise limit switch")
  private boolean getCCWLimitSwitchEnabled() {
    return m_talon.isFwdLimitSwitchClosed() == 1 ? true : false;
  }

  @Log(name = "Clockwise limit switch")
  private boolean getCWLimitSwitchEnabled() {
    return m_talon.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public void periodic() {
    double controllerVoltage =
        m_controller.calculate(this.getAngleRadians(), m_angleSetpointRadians);

    double feedforwardVoltage =
        m_feedforward.calculate(
            m_previousVelocitySetpoint,
            m_controller.getSetpoint().velocity,
            Constants.kTimestepSeconds);

    m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;

    m_talon.set(
        ControlMode.PercentOutput,
        (controllerVoltage + feedforwardVoltage) / Constants.kNominalVoltage);
  }
}
