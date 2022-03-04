// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeArmConstants.*;

import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeArm extends SubsystemBase implements Loggable {

  private WL_SparkMax m_spark;
  private SparkMaxLimitSwitch m_topLimit =
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
  private SparkMaxLimitSwitch m_bottomSwitch =
      m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  private final SR_ProfiledPIDController m_pidController =
      new SR_ProfiledPIDController(
          kPIntakeArmVoltsPerRadian,
          0,
          kDIntakeArmVoltSecondsPerRadian,
          kIntakeArmMotionProfileConstraints);

  private final SR_ArmFeedforward m_feedforward =
      new SR_ArmFeedforward(
          kSIntakeArmVolts,
          kGIntakeArmVolts,
          kVIntakeArmVoltsSecondsPerRadian,
          kAIntakeArmVoltsSecondsSquaredPerRadian);

  @Log(name = "angle setpoint radians")
  private double m_angleSetpointRadians = kIntakeArmBottomPositionRadians;

  private double m_lastVelocitySetpoint = 0;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  private DoubleLogEntry statorCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/statorCurrent");
  private DoubleLogEntry supplyCurrentLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/current/hood/supplyCurrent");

  /** Creates a new intakeArm. */
  public IntakeArm() {
    m_spark = new WL_SparkMax(kIntakeArmSparkPort);
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIntakeArmSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIntakeArmImmediateCurrentLimitAmps);

    m_topLimit.enableLimitSwitch(true);
    m_bottomSwitch.enableLimitSwitch(true);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_spark.getEncoder().getPosition() * kIntakeArmRadiansPerMotorRev;
  }

  @Config(name = "Set angle (radians)", defaultValueNumeric = kIntakeArmBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kIntakeArmBottomPositionRadians, kIntakeArmTopPositionRadians);
  }

  public void resetAngleRadians(double angle) {
    m_spark.getEncoder().setPosition(angle / kIntakeArmRadiansPerMotorRev);
  }

  @Log(name = "At goal")
  public boolean atGoal() {
    return this.getAngleRadians() - m_angleSetpointRadians < kIntakeArmPositionToleranceRadians;
  }

  @Log(name = "Top Limit Switch")
  public boolean getTopLimitSwitch() {
    return m_topLimit.isPressed();
  }

  @Log(name = "Bottom Limit Switch")
  public boolean getBottomLimitSwitch() {
    return m_bottomSwitch.isPressed();
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
              kIntakeArmLoopTimeSeconds);

      m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;

      m_spark.setVoltage(feedbackOutputVoltage + feedforwardOutputVoltage);
    }

    if (this.getBottomLimitSwitch()) {
      this.resetAngleRadians(kIntakeArmBottomPositionRadians);
    } else if (this.getTopLimitSwitch()) {
      this.resetAngleRadians(kIntakeArmTopPositionRadians);
    }

    statorCurrentLog.append(m_spark.getOutputCurrent());
    supplyCurrentLog.append(m_spark.getSupplyCurrent());
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
