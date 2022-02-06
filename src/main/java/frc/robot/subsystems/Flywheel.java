// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FlywheelConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Flywheel extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kFlywheelTalonPort);

  @Config(name = "Flywheel feedforward")
  private final SR_SimpleMotorFeedforward m_flywheelFeedforward =
      new SR_SimpleMotorFeedforward(kS, kV, kA);

  private final BangBangController m_bangBangController =
      new BangBangController(kVelocityTolerance);

  private double m_desiredVelocityRPS;

  /** Creates a new Flywheel. Controlled with a feedforward and a bang bang controlller. */
  public Flywheel() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.supplyCurrLimit.currentLimit = kFlywheelTalonCurrentLimit;
    flywheelTalonConfig.supplyCurrLimit.enable = true;
    flywheelTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    m_talon.configAllSettings(flywheelTalonConfig);

    m_talon.setNeutralMode(NeutralMode.Coast);
  }

  /** @return the current talon-reported velocity in rotations per second. */
  @Log(name = "Current Talon-reported Velocity (RPS)")
  public double getTalonVelocity() {
    return m_talon.getSelectedSensorVelocity() * kFlywheelRotationsPerPulse * 10;
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
  public void fastPeriodic() {
    // Calculates voltage to apply.
    // Feedforward is scaled down to prevent overshoot since bang-bang can't correct for overshoot.
    double voltage =
        0.95 * m_flywheelFeedforward.calculate(m_desiredVelocityRPS)
            + m_bangBangController.calculate(getTalonVelocity(), m_desiredVelocityRPS)
                * kNominalVoltage;
    m_talon.setVoltage(voltage);

    SmartDashboard.putNumber("ff applied voltage", voltage);
    SmartDashboard.putNumber("talon applied voltage", m_talon.getBusVoltage());
  }
}
