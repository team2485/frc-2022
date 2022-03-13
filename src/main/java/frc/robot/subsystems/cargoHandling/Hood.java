package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

  @Log(name = "Output voltage")
  private double m_lastOutputVoltage = 0;

  private boolean m_enabled = true;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(10);
    m_spark.setSecondaryCurrentLimit(15);
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

    if (Math.abs(angle - m_angleSetpointRadians) >= kHoodSetpointDeadbandRadians) {
      m_angleSetpointRadians =
          MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);
      if (angle > kHoodBottomPositionRadians && angle != m_angleSetpointRadians) {
        m_isZeroed = false;
      }
    }
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kHoodRadiansPerMotorRev);
  }

  @Log(name = "At goal")
  public boolean atGoal() {
    return Math.abs(this.getAngleRadians() - m_angleSetpointRadians)
        < kHoodPositionToleranceRadians;
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

  public void enable(boolean enabled) {
    m_enabled = enabled;
    if (m_enabled) {
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // default 10
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); // default 20
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // default 20
    } else {
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535); // default 10
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535); // default 20
      m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // default 20
    }
  }

  public void runControlLoop() {
    double outputVoltage = 0;
    if (m_enabled) {
      if (m_voltageOverride) {
        outputVoltage = m_voltageSetpoint;
      } else {
        if (m_angleSetpointRadians <= kHoodBottomPositionRadians || !m_isZeroed) {
          if (!this.getBottomLimitSwitch()) {
            outputVoltage = -3;
          }
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

          if (!this.atGoal()) {
            outputVoltage = m_feedbackOutput + m_feedforwardOutput;
          }
        }
      }
      if (this.getBottomLimitSwitch()) {
        zeroAngle();
        m_isZeroed = true;
      }
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
