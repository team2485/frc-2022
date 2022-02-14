package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.SparkMaxLimitSwitch;
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

  @Config(name = "Hood Controller")
  private final SR_ProfiledPIDController m_controller =
      new SR_ProfiledPIDController(kPHood, 0, kDHood, kHoodMotionProfileConstraints);

  @Config(name = "Hood Feedforward")
  private final SR_ArmFeedforward m_feedforward =
      new SR_ArmFeedforward(
          kSHoodVolts, kGHoodVolts, kVHoodVoltSecondsPerRadian, kAHoodVoltSecondsSquaredPerRadian);

  private double m_angleSetpointRadians = 0;
  private double m_previousVelocitySetpoint = 0;

  private boolean m_isZeroed = false;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodCurrentLimitAmps);

    m_spark.setInverted(true);

    m_limitSwitch.enableLimitSwitch(true);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kHoodRadiansPerMotorRev
        + kHoodBottomPositionRadians;
  }

  @Config(name = "Set angle (radians)")
  public void setAngleRadians(double angle) {
    m_angleSetpointRadians = angle;
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
