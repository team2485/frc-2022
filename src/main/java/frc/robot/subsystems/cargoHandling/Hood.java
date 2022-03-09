package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class Hood extends SubsystemBase implements Loggable {

  private final WL_SparkMax m_spark = new WL_SparkMax(kHoodSparkPort);
  private SparkMaxLimitSwitch m_limitSwitch =
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  private final Debouncer m_limitDebounce = new Debouncer(0.1, DebounceType.kBoth);
  private final SR_ProfiledPIDController m_pidController =
      new SR_ProfiledPIDController(kPHood, 0, kDHood, kHoodMotionProfileConstraints);

  private final SR_ArmFeedforward m_feedforward =
      new SR_ArmFeedforward(
          ksHoodVolts, kgHoodVolts, kvHoodVoltSecondsPerRadian, kaHoodVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  private double m_angleSetpointRadians = kHoodBottomPositionRadians;

  private double m_lastVelocitySetpoint = 0;

  @Log(name = "zeroed")
  private boolean m_isZeroed = false;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name = "Feedforward Output")
  private double m_feedforwardOutput;

  @Log(name = "Feedback Output")
  private double m_feedbackOutput;

  private DoubleLogEntry statorCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/statorCurrent");
  private DoubleLogEntry supplyCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/supplyCurrent");

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kHoodImmediateCurrentLimitAmps);

    m_spark.setInverted(true);

    m_spark.setIdleMode(IdleMode.kBrake);

    m_limitSwitch.enableLimitSwitch(true);

    m_pidController.setTolerance(kHoodPositionToleranceRadians);

    this.resetAngleRadians(kHoodBottomPositionRadians);

    Shuffleboard.getTab("Hood").add("Hood controller", m_pidController);
    Shuffleboard.getTab("Hood").add("Hood feedforward", m_feedforward);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kHoodRadiansPerMotorRev;
  }

  @Config(name = "Set angle (radians)", defaultValueNumeric = kHoodBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kHoodRadiansPerMotorRev);
  }

  @Log(name = "At goal")
  public boolean atGoal() {
    return this.getAngleRadians() - m_angleSetpointRadians < kHoodPositionToleranceRadians;
  }

  public void zeroAngle() {
    this.resetAngleRadians(kHoodBottomPositionRadians);
    m_isZeroed = true;
  }

  @Log(name = "Bottom Limit Switchs")
  public boolean getBottomLimitSwitch() {
    return m_limitDebounce.calculate(m_limitSwitch.isPressed());
  }

  @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void runControlLoop() {
    if (m_voltageOverride) {
      m_spark.setVoltage(m_voltageSetpoint);
    } else {
      double feedbackOutputVoltage =
          m_pidController.calculate(this.getAngleRadians(), m_angleSetpointRadians);

      double feedforwardOutputVoltage =
          m_feedforward.calculate(
              m_angleSetpointRadians,
              m_lastVelocitySetpoint,
              m_pidController.getSetpoint().velocity,
              kHoodLoopTimeSeconds);

      m_feedbackOutput = feedbackOutputVoltage;
      m_feedforwardOutput = feedforwardOutputVoltage;
      m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;

      m_spark.setVoltage(feedbackOutputVoltage + feedforwardOutputVoltage);
    }

    if (this.getBottomLimitSwitch()) {
      zeroAngle();
    }

    statorCurrentLog.append(m_spark.getOutputCurrent());
    supplyCurrentLog.append(m_spark.getSupplyCurrent());
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
