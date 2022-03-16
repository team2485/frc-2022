// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeArmConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeArm extends SubsystemBase implements Loggable {

  private final WL_SparkMax m_spark = new WL_SparkMax(kIntakeArmSparkPort);
  private final SparkMaxLimitSwitch m_topLimitSwitch =
      m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
  private final SparkMaxLimitSwitch m_bottomLimitSwitch =
      m_spark.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(4);
  private final Debouncer m_topLimitDebounce = new Debouncer(0.1, DebounceType.kBoth);
  private final Debouncer m_bottomLimitDebounce = new Debouncer(0.1, DebounceType.kBoth);
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

  @Log(name = "Feedforward Output")
  private double m_feedforwardOutput = 0;

  @Log(name = "Feedback Output")
  private double m_feedbackOutput = 0;

  @Log(name = "Setpoint position")
  private boolean m_armSetpointPosition = true; // true up, false down

  @Log(name = "Current position")
  private boolean m_armPosition = true;

  private double m_lastStatorCurrent = 0;

  @Log(name = "Output Voltage")
  private double m_lastOutputVoltage = 0;

  /** Creates a new intakeArm. */
  public IntakeArm() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIntakeArmSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIntakeArmImmediateCurrentLimitAmps);
    m_topLimitSwitch.enableLimitSwitch(true);
    m_bottomLimitSwitch.enableLimitSwitch(true);
    m_spark.setIdleMode(IdleMode.kBrake);

    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535); // default 10
    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535); // default 20
    m_spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535); // default 20

    Shuffleboard.getTab("IntakeArm").add("controller", m_pidController);
    Shuffleboard.getTab("IntakeArm").add("feedforward", m_feedforward);
  }

  /** @return current angle from horizontal */
  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return -m_encoder.getAbsolutePosition() * 2 * Math.PI + kIntakeArmEncoderOffset;
  }

  @Config(name = "Set angle (radians)", defaultValueNumeric = kIntakeArmBottomPositionRadians)
  public void setAngleRadians(double angle) {
    m_voltageOverride = false;
    m_angleSetpointRadians =
        MathUtil.clamp(angle, kIntakeArmBottomPositionRadians, kIntakeArmTopPositionRadians);
  }

  @Log(name = "At goal")
  public boolean atGoal() {
    return this.getAngleRadians() - m_angleSetpointRadians < kIntakeArmPositionToleranceRadians;
  }

  @Log(name = "Top Limit Switch")
  public boolean getTopLimitSwitch() {
    return m_topLimitDebounce.calculate(m_topLimitSwitch.isPressed());
  }

  @Log(name = "Bottom Limit Switch")
  public boolean getBottomLimitSwitch() {
    return m_bottomLimitDebounce.calculate(m_bottomLimitSwitch.isPressed());
  }

  @Config.NumberSlider(name = "Set voltage", min = -12, max = 12)
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  @Config.ToggleSwitch(name = "Set arm position", defaultValue = true)
  public void setPosition(boolean top) {
    m_voltageOverride = false;
    m_armSetpointPosition = top;
  }

  public boolean atPosition(boolean top) {
    if (top) {
      return Math.abs(this.getAngleRadians() - kIntakeArmTopPositionRadians)
          < kIntakeArmPositionToleranceRadians;
    } else {
      return Math.abs(this.getAngleRadians() - kIntakeArmBottomPositionRadians)
          < kIntakeArmPositionToleranceRadians;
    }
  }

  public void runControlLoop() {
    if (this.atPosition(true)) {
      m_armPosition = true;
    } else if (this.atPosition(false)) {
      m_armPosition = false;
    }

    if (m_voltageOverride) {
      m_spark.setVoltage(m_voltageSetpoint);
    } else {
      double outputVoltage = 0;
      if (m_armSetpointPosition && !m_armPosition) {
        // System.out.println("Going up");
        if (this.getAngleRadians() > 1.6) {
          outputVoltage = 0;
        } else {
          outputVoltage = 6;
        }
      } else if (!m_armSetpointPosition && this.getAngleRadians() > -0.12) {
        // System.out.println("Going down");

        if (this.getAngleRadians() > 1.6) {
          outputVoltage = -6;
        } else {
          outputVoltage = -2;
        }
      }

      if (outputVoltage != m_lastOutputVoltage) {
        m_spark.setVoltage(outputVoltage);
      }
      m_lastOutputVoltage = outputVoltage;
    }
  }

  @Override
  public void periodic() {
    this.runControlLoop();
  }
}
