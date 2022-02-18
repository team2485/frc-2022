package frc.robot.subsystems;

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
  private SparkMaxLimitSwitch m_limitSwitch =
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private final SR_ProfiledPIDController m_controller =
      new SR_ProfiledPIDController(kPHood, 0, kDHood, kHoodMotionProfileConstraints);

  @Log(name = "Hood Feedforward")
  private final SR_ArmFeedforward m_feedforward =
      new SR_ArmFeedforward(
          kSHoodVolts, kGHoodVolts, kVHoodVoltSecondsPerRadian, kAHoodVoltSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  private double m_angleSetpointRadians = kHoodBottomPositionRadians;

  private double m_previousVelocitySetpoint = 0;

  private boolean m_isZeroed = false;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kHoodImmediateCurrentLimitAmps);

    m_spark.setInverted(true);

    m_spark.setIdleMode(IdleMode.kBrake);

    m_limitSwitch.enableLimitSwitch(true);

    m_controller.setTolerance(kHoodControllerPositionTolerance);

    this.resetAngleRadians(kHoodBottomPositionRadians);

    Shuffleboard.getTab("Hood").add("Hood controller", m_controller);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kHoodRadiansPerMotorRev;
  }

  @Config(name = "Set angle (radians)", defaultValueNumeric = kHoodBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kHoodRadiansPerMotorRev);
  }

  public boolean getBottomLimitSwitch() {
    return m_limitSwitch.isPressed();
  }

  @Override
  public void periodic() {
    double controllerVoltage =
        m_controller.calculate(this.getAngleRadians(), m_angleSetpointRadians);

    double feedforwardVoltage =
        m_feedforward.calculate(
            m_angleSetpointRadians,
            m_previousVelocitySetpoint,
            m_controller.getSetpoint().velocity,
            Constants.kTimestepSeconds);

    m_previousVelocitySetpoint = m_controller.getSetpoint().velocity;

    m_spark.set((controllerVoltage + feedforwardVoltage) / Constants.kNominalVoltage);
  }
}
