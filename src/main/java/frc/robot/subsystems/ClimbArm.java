package frc.robot.subsystems;

import static frc.robot.Constants.ClimbArmConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_ArmFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ElevatorFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class ClimbArm extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_armMotor = new WPI_TalonFX(kArmTalonPort);

  public enum ArmMode {
    kTranslation,
    kRotation
  }

  private ArmMode m_armMode = ArmMode.kRotation;

  private double m_armAngleRadians = 0; // from vertical down
  private double m_armAngleSetpointRadians = 0;
  private double m_lastAngularVelocitySetpointRotation = 0;

  // input of angle error, output of current
  private final SR_ProfiledPIDController m_armControllerRotation =
      new SR_ProfiledPIDController(
          kPArmRotationAmpsPerRadian,
          0,
          kDArmRotationAmpSecondsPerRadian,
          kArmControllerConstraintsRotation);

  // input of angular accel, output of current
  private final SR_ArmFeedforward m_armFeedforwardRotation =
      new SR_ArmFeedforward(
          ksArmRotationAmps,
          kgArmRotationAmps,
          kvArmRotationAmpSecondsPerRadian,
          kaArmRotationAmpSecondsSquaredPerRadian);

  // input of position error, output of voltage
  private final SR_ProfiledPIDController m_armControllerTranslation =
      new SR_ProfiledPIDController(
          kPArmTranslationVoltsPerMeter,
          0,
          kDArmTranslationVoltSecondsPerMeter,
          kArmControllerConstraintsTranslation);

  // input of acceleration, output of voltage
  private final SR_ElevatorFeedforward m_armFeedforwardTranslation =
      new SR_ElevatorFeedforward(
          ksArmTranslationVolts,
          kgArmTranslationVolts,
          kvArmTranslationVoltSecondsPerMeter,
          kaArmTranslationVoltSecondsSquaredPerMeter);

  private double m_translationSetpointMeters = 0;
  private double m_lastVelocitySetpointTranslation = 0;

  public ClimbArm() {
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    armMotorConfig.supplyCurrLimit.currentLimit = kArmCurrentLimitAmps;
    armMotorConfig.supplyCurrLimit.enable = true;
    armMotorConfig.slot0.kP = kPArmCurrentVoltsPerAmp;
    armMotorConfig.slot0.kD = kDArmCurrentVoltSecondsPerAmp;
    m_armMotor.configAllSettings(armMotorConfig);
    m_armMotor.enableVoltageCompensation(true);
    m_armMotor.setNeutralMode(NeutralMode.Brake);

    m_armControllerRotation.setTolerance(kArmRotationToleranceRadians);
    m_armControllerTranslation.setTolerance(kArmTranslationToleranceMeters);

    this.resetAbsoluteRotation(0);
  }

  public void setArmControlMode(ArmMode mode) {
    m_armMode = mode;
    this.resetAbsoluteRotation(0);
  }

  public void resetAbsoluteRotation(double rotations) {
    m_armMotor.setSelectedSensorPosition(rotations / kArmRotationsPerPulse);
  }

  public double getAbsoluteRotation() {
    return m_armMotor.getSelectedSensorPosition() * kArmRotationsPerPulse;
  }

  public double getAngleRadians() {
    return this.getAbsoluteRotation() * 2 * Math.PI;
  }

  public void setAngleRadians(double angle) {
    m_armAngleSetpointRadians = angle;
  }

  public double getTranslationMeters() {
    return this.getAbsoluteRotation() * kSprocketCircumferenceMeters;
  }

  public void setTranslationMeters(double translation) {
    m_translationSetpointMeters = translation;
  }

  @Config(name = "Arm Current PD Terms")
  private void setCurrentPDTerms(double kP, double kD) {
    m_armMotor.config_kP(0, kP);
    m_armMotor.config_kD(0, kD);
  }

  public void runControlLoop() {
    if (m_armMode == ArmMode.kRotation) {
      double feedbackOutputCurrent =
          m_armControllerRotation.calculate(getAngleRadians(), m_armAngleSetpointRadians);

      double feedforwardOutputCurrent =
          m_armFeedforwardRotation.calculate(
              m_lastAngularVelocitySetpointRotation,
              m_armControllerRotation.getSetpoint().velocity,
              kArmControlLoopTimeSeconds);

      m_armMotor.set(ControlMode.Current, feedforwardOutputCurrent + feedbackOutputCurrent);

      m_lastAngularVelocitySetpointRotation = m_armControllerRotation.getSetpoint().velocity;
    } else if (m_armMode == ArmMode.kTranslation) {
      double feedbackOutputVoltage =
          m_armControllerTranslation.calculate(getTranslationMeters(), m_translationSetpointMeters);

      double feedforwardOutputVoltage =
          m_armFeedforwardTranslation.calculate(
              m_lastVelocitySetpointTranslation,
              m_armControllerTranslation.getSetpoint().velocity,
              kArmControlLoopTimeSeconds);

      double outputPercentage =
          (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;

      m_armMotor.set(ControlMode.PercentOutput, outputPercentage);

      m_lastVelocitySetpointTranslation = m_armControllerTranslation.getSetpoint().velocity;
    }
  }

  @Override
  public void periodic() {
    // this.runControlLoop();
  }
}
