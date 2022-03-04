package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Feeder extends SubsystemBase implements Loggable {
  private WL_SparkMax m_spark = new WL_SparkMax(kFeederSparkPort);

  private Servo m_servo = new Servo(kFeederServoPort);
  private double m_servoPositionSetpoint = 0;

  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
          kSFeederVolts, kVFeederVoltSecondsPerMeter, kAFeederVoltSecondsSquaredPerMeter);

  @Log(name = "Velocity Setpoint")
  private double m_velocitySetpointRotationsPerSecond;

  private double m_lastVelocitySetpoint;

  @Log(name = "Feedforward output")
  private double m_feedforwardOutput;

  private DoubleLogEntry statorCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/feeder/statorCurrent");
  private DoubleLogEntry supplyCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/feeder/supplyCurrent");

  public Feeder() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kFeederSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kFeederImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
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
    m_velocitySetpointRotationsPerSecond = rotationsPerSecond;
  }

  /**
   * Applys the given voltage to the talon.
   *
   * @param voltage what voltage to apply
   */
  public void setVoltage(double voltage) {
    m_spark.setVoltage(voltage);
  }

  @Log(name = "At setpoint")
  public boolean atSetpoint() {
    return Math.abs(getVelocityRotationsPerSecond() - m_velocitySetpointRotationsPerSecond)
        < kFeederVelocityToleranceRotationsPerSecond;
  }

  public void setServo(double position) {
    m_servoPositionSetpoint = position;
  }

  @Config.ToggleButton(name = "Set servo")
  public void engageServo(boolean engaged) {
    if (engaged) {
      m_servoPositionSetpoint = kServoEngagedPosition;
    } else {
      m_servoPositionSetpoint = kServoDisengagedPosition;
    }
  }

  public void runControlLoop() {
    // Calculates voltage to apply.

    double feedforwardOutput =
        m_feedforward.calculate(
            m_lastVelocitySetpoint, m_velocitySetpointRotationsPerSecond, kFeederLoopTimeSeconds);

    m_spark.setVoltage(feedforwardOutput);

    m_feedforwardOutput = feedforwardOutput;

    statorCurrentLog.append(m_spark.getOutputCurrent());
    supplyCurrentLog.append(m_spark.getSupplyCurrent());
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
