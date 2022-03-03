package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
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
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

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

  DoubleLogEntry statorCurrentLog;
  DoubleLogEntry supplyCurrentLog;

  public Hood() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kHoodSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kHoodImmediateCurrentLimitAmps);

    m_spark.setInverted(true);

    m_spark.setIdleMode(IdleMode.kBrake);

    m_limitSwitch.enableLimitSwitch(true);

    m_pidController.setTolerance(kHoodControllerPositionTolerance);

    this.resetAngleRadians(kHoodBottomPositionRadians);

    statorCurrentLog = new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/statorCurrent");
    supplyCurrentLog = new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/supplyCurrent");

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
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kHoodBottomPositionRadians, kHoodTopPositionRadians);

    m_pidController.setGoal(
        m_angleSetpointRadians); // need to do this here because otherwise atGoal will return true
    // briefly before the new goal is set in periodic
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

  public boolean getBottomLimitSwitch() {
    return m_limitSwitch.isPressed();
  }

  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }

  public void runControlLoop() {
    double feedbackOutputVoltage =
        m_pidController.calculate(
            this.getAngleRadians()); // feedback controller already has goal from setAngleRadians
    // method

    double feedforwardOutputVoltage =
        m_feedforward.calculate(
            m_angleSetpointRadians,
            m_lastVelocitySetpoint,
            m_pidController.getSetpoint().velocity,
            kHoodLoopTimeSeconds);

    m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;

    m_spark.setVoltage(feedbackOutputVoltage + feedforwardOutputVoltage);

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
