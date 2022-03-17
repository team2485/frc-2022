package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Feeder extends SubsystemBase implements Loggable {
  private final WL_SparkMax m_spark = new WL_SparkMax(kFeederSparkPort);

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
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kFeederSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kFeederImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535); // default 10
    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535); // default 20
    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // default 20
  }

  /** @return the current velocity in rotations per second. */
  @Log(name = "Current velocity (RPS)")
  public double getVelocityRotationsPerSecond() {
    return m_spark.getEncoder().getVelocity() / 60.0;
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
      m_spark.setVoltage(outputVoltage);
    }

    m_lastOutputVoltage = outputVoltage;
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
