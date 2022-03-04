// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kShooterTalonPort);

  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
          kSShooterVolts, kVShooterVoltSecondsPerMeter, kAShooterVoltSecondsSquaredPerMeter);

  private final BangBangController m_bangBangController =
      new BangBangController(kVelocityTolerance);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond = 0;

  private double m_lastVelocitySetpoint = 0;

  @Log(name = "Feedback output")
  private double m_feedbackOutput;

  @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private double m_lastVelocity = 0;

  DoubleLogEntry statorCurrentLog;
  DoubleLogEntry supplyCurrentLog;

  /** Creates a new Shooter. Controlled with a feedforward and a bang bang controlller. */
  public Shooter() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.supplyCurrLimit.currentLimit = kShooterTalonCurrentLimit;
    talonConfig.supplyCurrLimit.enable = true;
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    m_talon.configAllSettings(talonConfig);

    m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    m_talon.setNeutralMode(NeutralMode.Coast);
    m_talon.enableVoltageCompensation(true);

    statorCurrentLog =
        new DoubleLogEntry(DataLogManager.getLog(), "/current/shooter/statorCurrent");
    supplyCurrentLog =
        new DoubleLogEntry(DataLogManager.getLog(), "/current/shooter/supplyCurrent");

    Shuffleboard.getTab("Shooter").add("Feedforward", m_feedforward);
  }

  /** @return the current talon-reported velocity in rotations per second. */
  @Log(name = "Current velocity (RPS)")
  public double getVelocityRotationsPerSecond() {
    return m_talon.getSelectedSensorVelocity() * kShooterRotationsPerPulse * 10;
  }

  /**
   * Sets the velocity setpoint for the flywheel.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_lastVelocitySetpoint = m_velocitySetpointRotationsPerSecond;
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  public void setVoltage(double voltage) {
    m_talon.setVoltage(voltage);
  }

  @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kVelocityTolerance;
  }

  public boolean hasDipped() {
    return m_lastVelocity - this.getVelocityRotationsPerSecond()
            > kShooterVelocityDipThresholdRotationsPerSecond
        && m_velocitySetpointRotationsPerSecond >= m_lastVelocitySetpoint;
  }

  // runs every 10 ms (run by Robot)
  public void runControlLoop() {
    // Calculates voltage to apply.
    // Feedforward is scaled down to prevent overshoot since bang-bang can't correct for overshoot.
    double feedforwardOutput =
        kShooterFeedforwardScale * m_feedforward.calculate(m_velocitySetpointRotationsPerSecond);
    double feedbackOutput =
        this.atSetpoint()
            ? 0
            : m_bangBangController.calculate(
                    this.getVelocityRotationsPerSecond(), m_velocitySetpointRotationsPerSecond)
                * kNominalVoltage;

    double outputVoltage = feedforwardOutput + feedbackOutput;
    m_talon.set(ControlMode.PercentOutput, outputVoltage / kNominalVoltage);

    m_feedbackOutput = feedbackOutput;
    m_feedforwardOutput = feedforwardOutput;

    m_lastVelocity = this.getVelocityRotationsPerSecond();

    statorCurrentLog.append(m_talon.getStatorCurrent());
    supplyCurrentLog.append(m_talon.getSupplyCurrent());
  }

  @Override
  public void periodic() {}
}
