package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeederConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Feeder extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kFeederTalonPort);

  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
          kSFeederVolts, kVFeederVoltSecondsPerMeter, kAFeederVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  // @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name = "Output voltage")
  private double m_lastOutputVoltage = 0;

  public Feeder() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kFeederSupplyCurrentLimitAmps,
            kFeederSupplyCurrentThresholdAmps,
            kFeederSupplyCurrentThresholdTimeSecs);
    talonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kFeederStatorCurrentLimitAmps,
            kFeederStatorCurrentThresholdAmps,
            kFeederStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(talonConfig);

    m_talon.setInverted(true);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);
  }

  /** @return the current velocity in rotations per second. */
  @Log(name = "Current velocity (RPS)")
  public double getVelocityRotationsPerSecond() {
    return m_talon.getSelectedSensorVelocity()
        / (kFeederGearRatio * kFalconSensorUnitsPerRotation * 10);
  }

  /**
   * Sets the velocity setpoint for the feeeder.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;

    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  @Config.NumberSlider(name = "Set Voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  // @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kFeederVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    double outputVoltage = 0;
    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput =
          m_feedforward.calculate(
              m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond, kFeederLoopTimeSeconds);

      outputVoltage = feedforwardOutput;

      m_feedforwardOutput = feedforwardOutput;
    }
    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(outputVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
