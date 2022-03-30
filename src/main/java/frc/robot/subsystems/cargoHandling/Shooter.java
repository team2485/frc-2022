// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_shooterTalon = new WPI_TalonFX(kShooterTalonPort);

  private final WPI_TalonFX m_kickerTalon = new WPI_TalonFX(kKickerTalonPort);

  @Log(name = "Shooter velocity Setpoint")
  private double m_shooterVelocitySetpointRotationsPerSecond = 0;

  @Log(name = "Kicker velocity Setpoint")
  private double m_kickerVelocitySetpointRotationsPerSecond = 0;

  /** Creates a new Shooter. Controlled with a feedforward and a bang bang controlller. */
  public Shooter() {
    TalonFXConfiguration shooterTalonConfig = new TalonFXConfiguration();
    shooterTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    shooterTalonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kShooterSupplyCurrentLimitAmps,
            kShooterSupplyCurrentThresholdAmps,
            kShooterSupplyCurrentThresholdTimeSecs);
    shooterTalonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kShooterStatorCurrentLimitAmps,
            kShooterStatorCurrentThresholdAmps,
            kShooterStatorCurrentThresholdTimeSecs);

    shooterTalonConfig.peakOutputReverse = 0;
    shooterTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    shooterTalonConfig.velocityMeasurementWindow = 1;

    shooterTalonConfig.slot0.kP = kPShooterOutputUnit100MsPerSensorUnit;
    shooterTalonConfig.slot0.kF = kFShooterOutputUnit100MsPerSensorUnit * kShooterFeedforwardScale;
    shooterTalonConfig.slot0.allowableClosedloopError =
        kShooterControlVelocityToleranceSensorUnitsPer100Ms;

    m_shooterTalon.configAllSettings(shooterTalonConfig);
    m_shooterTalon.setNeutralMode(NeutralMode.Coast);
    m_shooterTalon.enableVoltageCompensation(true);

    TalonFXConfiguration kickerTalonConfig = new TalonFXConfiguration();
    kickerTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    kickerTalonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kKickerSupplyCurrentLimitAmps,
            kKickerSupplyCurrentThresholdAmps,
            kKickerSupplyCurrentThresholdTimeSecs);
    kickerTalonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kKickerStatorCurrentLimitAmps,
            kKickerStatorCurrentThresholdAmps,
            kKickerStatorCurrentThresholdTimeSecs);

    kickerTalonConfig.peakOutputReverse = 0;
    kickerTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    kickerTalonConfig.velocityMeasurementWindow = 1;

    kickerTalonConfig.slot0.kP = kPKickerOutputUnit100MsPerSensorUnit;
    kickerTalonConfig.slot0.kF = kFKickerOutputUnit100MsPerSensorUnit * kKickerFeedforwardScale;
    kickerTalonConfig.slot0.allowableClosedloopError =
        kKickerControlVelocityToleranceSensorUnitsPer100Ms;

    m_kickerTalon.configAllSettings(kickerTalonConfig);
    m_kickerTalon.setNeutralMode(NeutralMode.Coast);
    m_kickerTalon.enableVoltageCompensation(true);

    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    m_shooterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    m_kickerTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);
  }

  /** @return the current talon-reported shooter velocity in rotations per second. */
  @Log(name = "Current shooter velocity (RPS)")
  public double getShooterVelocityRotationsPerSecond() {
    return m_shooterTalon.getSelectedSensorVelocity()
        / kShooterGearRatio
        / kFalconSensorUnitsPerRotation
        * 10;
  }

  /** @return the current talon-reported shooter velocity in rotations per second. */
  @Log(name = "Current kicker velocity (RPS)")
  public double getKickerVelocityRotationsPerSecond() {
    return m_kickerTalon.getSelectedSensorVelocity()
        / kKickerGearRatio
        / kFalconSensorUnitsPerRotation
        * 10;
  }

  /**
   * Sets shooter and kicker wheel velocities.
   *
   * @param shooterVelocityRotationsPerSecond desired shooter velocity
   * @param tangentialVelocityRatio desired kicker wheel tangential velocity divided by shooter
   *     wheel tangential velocity (m/s) 1/3 will make angular velocities equal, >1 creates topspin,
   *     <1 creates backspin
   */
  @Config(name = "Set Velocities")
  public void setVelocities(
      double shooterVelocityRotationsPerSecond, double tangentialVelocityRatio) {
    this.setShooterVelocityRotationsPerSecond(shooterVelocityRotationsPerSecond);
    this.setKickerVelocityRotationsPerSecond(
        shooterVelocityRotationsPerSecond
            * kShooterCircumferenceMeters // desired shooter tangential velocity m/s
            * tangentialVelocityRatio // desired kicker tangential velocity m/s
            / kKickerCircumferenceMeters); // desired kicker angular velocity rots/sec
  }

  /**
   * Sets shooter and kicker wheel velocities, with default tangential velocity ratio
   *
   * @param shooterVelocityRotationsPerSecond desired shooter velocity
   */
  public void setVelocities(double shooterVelocityRotationsPerSecond) {
    this.setVelocities(shooterVelocityRotationsPerSecond, kDefaultTangentialVelocityRatio);
  }

  /**
   * Sets the velocity setpoint for the shooter.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  @Config(name = "Set Shooter Velocity (RPS)")
  public void setShooterVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
    double newVelocitySetpointRotationsPerSecond =
        MathUtil.clamp(velocityRotationsPerSecond, 0, kShooterMaxSpeedRotationsPerSecond);
    m_shooterVelocitySetpointRotationsPerSecond = newVelocitySetpointRotationsPerSecond;
    m_shooterTalon.set(
        ControlMode.Velocity,
        newVelocitySetpointRotationsPerSecond
            * kFalconSensorUnitsPerRotation
            * kShooterGearRatio
            * 0.1,
        DemandType.ArbitraryFeedForward,
        velocityRotationsPerSecond > 0 ? kSShooterVolts / kNominalVoltage : 0);
  }

  /**
   * Sets the velocity setpoint for the kicker.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  @Config(name = "Set Kicker Velocity (RPS)")
  public void setKickerVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
    double newVelocitySetpointRotationsPerSecond =
        MathUtil.clamp(velocityRotationsPerSecond, 0, 180);
    if (newVelocitySetpointRotationsPerSecond != m_kickerVelocitySetpointRotationsPerSecond) {
      m_kickerVelocitySetpointRotationsPerSecond = newVelocitySetpointRotationsPerSecond;
      m_kickerTalon.set(
          ControlMode.Velocity,
          newVelocitySetpointRotationsPerSecond
              * kFalconSensorUnitsPerRotation
              * kKickerGearRatio
              * 0.1,
          DemandType.ArbitraryFeedForward,
          velocityRotationsPerSecond > 0 ? kSKickerVolts / kNominalVoltage : 0);
    }
  }

  /**
   * Applies the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  @Config(name = "Set shooter voltage")
  public void setShooterVoltage(double voltage) {
    m_shooterTalon.set(ControlMode.PercentOutput, voltage / kNominalVoltage);
  }

  /**
   * Applies the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  @Config(name = "Set kicker voltage")
  public void setKickerVoltage(double voltage) {
    m_kickerTalon.set(ControlMode.PercentOutput, voltage / kNominalVoltage);
  }

  public boolean shooterWithinTolerance(double tolerance) {
    return Math.abs(
            getShooterVelocityRotationsPerSecond() - m_shooterVelocitySetpointRotationsPerSecond)
        < tolerance;
  }

  public boolean kickerWithinTolerance(double tolerance) {
    return Math.abs(
            getKickerVelocityRotationsPerSecond() - m_kickerVelocitySetpointRotationsPerSecond)
        < tolerance;
  }

  public boolean withinTolerance(double tolerance) {
    return shooterWithinTolerance(tolerance) && kickerWithinTolerance(tolerance);
  }

  public double getSetpoint() {
    return m_shooterVelocitySetpointRotationsPerSecond;
  }

  public void periodic() {}
}
