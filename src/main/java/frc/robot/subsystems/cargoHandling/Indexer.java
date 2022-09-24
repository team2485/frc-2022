package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Indexer extends SubsystemBase implements Loggable {
  private WPI_TalonFX m_talon = new WPI_TalonFX(kIndexerTalonPort);

  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
          kSIndexerVolts, kVIndexerVoltSecondsPerMeter, kAIndexerVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  private double m_lastVelocity;

  // @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  private double m_lastOutputVoltage = 0;

  @Log(name = "output voltage")
  private double outputVoltage = 0;

  public Indexer() {

    TalonFXConfiguration indexerTalonConfig = new TalonFXConfiguration();
    indexerTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    indexerTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    indexerTalonConfig.velocityMeasurementWindow = 1;

    indexerTalonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kIndexerSupplyCurrentLimitAmps,
            kIndexerSupplyCurrentThresholdAmps,
            kIndexerSupplyCurrentThresholdTimeSecs);

    indexerTalonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kIndexerStatorCurrentLimitAmps,
            kIndexerStatorCurrentThresholdAmps,
            kIndexerStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(indexerTalonConfig);
    m_talon.setNeutralMode(NeutralMode.Brake);
    m_talon.setInverted(true);
    m_talon.enableVoltageCompensation(true);
  }

  /** @return the current velocity in rotations per second. */
  // @Log(name = "Current velocity (RPS)")
  @Log(name = "current rotations/sec")
  public double getVelocityRotationsPerSecond() {
    return m_talon.getSelectedSensorVelocity()
        / (kIndexerGearRatio * kFalconSensorUnitsPerRotation);
  }

  /**
   * Sets the velocity setpoint for the feeeder.
   *
   * @param rotationsPerSecond velocity setpoint
   */
  // @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_voltageOverride = false;
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  // @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  // @Log(name = "Stator current")
  // public double getStatorCurrent() {
  //   return m_talon.getOutputCurrent();
  // }

  // @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kIndexerVelocityToleranceRotationsPerSecond;
  }

  public void runControlLoop() {
    // Calculates voltage to apply.
    outputVoltage = 0;
    if (m_voltageOverride) {
      outputVoltage = m_voltageSetpoint;
    } else {
      double feedforwardOutput =
          m_feedforward.calculate(
              m_lastVelocitySetpoint,
              m_velocitySetpointRotationsPerSecond,
              kIndexerLoopTimeSeconds);

      outputVoltage = feedforwardOutput;

      m_feedforwardOutput = feedforwardOutput;
    }

    if (outputVoltage != m_lastOutputVoltage) {
      m_talon.setVoltage(outputVoltage);
    }
    m_lastOutputVoltage = outputVoltage;
    m_lastVelocity = this.getVelocityRotationsPerSecond();
  }

  public void periodic() {
    this.runControlLoop();
    ;
  }
}
