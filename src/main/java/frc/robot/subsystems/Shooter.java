// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.CurrentLogger;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {
  private final WL_TalonFX m_talon = new WL_TalonFX(kShooterTalonPort);

  @Config(name = "Flywheel feedforward")
  private final SR_SimpleMotorFeedforward m_shooterFeedforward =
      new SR_SimpleMotorFeedforward(kS, kV, kA);

  private final BangBangController m_bangBangController =
      new BangBangController(kVelocityTolerance);

  private double m_desiredVelocityRPS;

  /** Creates a new Shooter. Controlled with a feedforward and a bang bang controlller. */
  public Shooter() {
    CurrentLogger.getInstance().register(m_talon, "Shooter");
    TalonFXConfiguration shooterTalonConfig = new TalonFXConfiguration();
    shooterTalonConfig.supplyCurrLimit.currentLimit = kShooterTalonCurrentLimit;
    shooterTalonConfig.supplyCurrLimit.enable = true;
    shooterTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    m_talon.configAllSettings(shooterTalonConfig);

    m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    m_talon.setNeutralMode(NeutralMode.Coast);
    m_talon.enableVoltageCompensation(true);
  }

  /** @return the current talon-reported velocity in rotations per second. */
  @Log(name = "Current Talon-reported Velocity (RPS)")
  public double getTalonVelocity() {
    return m_talon.getSelectedSensorVelocity() * kShooterRotationsPerPulse * 10;
  }

  /**
   * Sets the velocity setpoint for the flywheel.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_desiredVelocityRPS = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  public void setVoltage(double voltage) {
    m_talon.setVoltage(voltage);
  }

  // runs every 10 ms (run by Robot)
  public void runShooterControlLoop() {
    // Calculates voltage to apply.
    // Feedforward is scaled down to prevent overshoot since bang-bang can't correct for overshoot.
    double voltage =
        0.95 * m_shooterFeedforward.calculate(m_desiredVelocityRPS)
            + m_bangBangController.calculate(getTalonVelocity(), m_desiredVelocityRPS)
                * kNominalVoltage;
    m_talon.set(ControlMode.PercentOutput, voltage / kNominalVoltage);

    SmartDashboard.putNumber("ff applied voltage", voltage);
    SmartDashboard.putNumber("talon applied voltage", m_talon.getBusVoltage());
  }

  @Override
  public void periodic() {}
}
