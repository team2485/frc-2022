package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
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

  private final SR_ProfiledPIDController m_pidController =
      new SR_ProfiledPIDController(kPHood, 0, kDHood, kHoodMotionProfileConstraints);

  private final SR_ArmFeedforward m_feedforward =
      new SR_ArmFeedforward(
          kSHoodVolts, kGHoodVolts, kVHoodVoltSecondsPerRadian, kAHoodVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  private double m_angleSetpointRadians = kHoodBottomPositionRadians;

  private double m_lastVelocitySetpoint = 0;

  @Log(name = "zeroed")
  private boolean m_isZeroed = false;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kHoodImmediateCurrentLimitAmps);

    m_spark.setInverted(true);

    m_spark.setIdleMode(IdleMode.kBrake);

    m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);

    m_pidController.setTolerance(kHoodControllerPositionTolerance);

    this.resetAngleRadians(kHoodBottomPositionRadians);

    Shuffleboard.getTab("Hood").add("Hood controller", m_pidController);
    Shuffleboard.getTab("Hood").add("Hood feedforward", m_feedforward);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kHoodRadiansPerMotorRev;
  }

  @Config.NumberSlider(
      name = "Set angle (radians)",
      min = kHoodBottomPositionRadians,
      max = kHoodTopPositionRadians,
      defaultValue = kHoodBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kHoodRadiansPerMotorRev);
  }

  public boolean atGoal() {
    return m_pidController.atGoal();
  }

  public void zeroAngle() {
    this.resetAngleRadians(kHoodBottomPositionRadians);
    m_isZeroed = true;
  }

  @Log(name = "bottom limit switch")
  public boolean getBottomLimitSwitch() {
    return m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
  }

  @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  @Override
  public void periodic() {
    if (m_voltageOverride) {
      m_spark.setVoltage(m_voltageSetpoint);
    } else {
      double feedbackOutputVoltage =
          m_pidController.calculate(
              this.getAngleRadians()); // feedback controller already has goal from setAngleRadians
      // method

      double feedforwardOutputVoltage =
          m_feedforward.calculate(
              m_angleSetpointRadians,
              m_lastVelocitySetpoint,
              m_pidController.getSetpoint().velocity,
              Constants.kTimestepSeconds);

      m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;

      m_spark.setVoltage(feedbackOutputVoltage + feedforwardOutputVoltage);
    }
    if (this.getBottomLimitSwitch()) {
      zeroAngle();
    }
  }
}
