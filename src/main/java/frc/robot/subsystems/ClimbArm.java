package frc.robot.subsystems;

import static frc.robot.Constants.ClimbArmConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private double m_lastArmVelocitySetpointRotation = 0;

  private final ProfiledPIDController m_armControllerRotation =
      new ProfiledPIDController(kPArmRotation, 0, kDArmRotation, kArmControllerConstraintsRotation);

  private final ProfiledPIDController m_armControllerTranslation =
      new ProfiledPIDController(
          kPArmTranslation, 0, kDArmTranslation, kArmControllerConstraintsTranslation);

  private double m_lastArmVelocitySetpointTranslation = 0;

  private final SimpleMotorFeedforward m_armFeedforwardTranslation =
      new SimpleMotorFeedforward(
          ksArmTranslationVolts,
          kvArmTranslationVoltSecondsPerMeter,
          kaArmTranslationVoltSecondsSquaredPerMeter);

  public ClimbArm() {

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    armMotorConfig.supplyCurrLimit.currentLimit = kArmCurrentLimitAmps;
    armMotorConfig.supplyCurrLimit.enable = true;
    armMotorConfig.slot0.kP = kPArmCurrent;
    armMotorConfig.slot0.kD = kDArmCurrent;
    m_armMotor.configAllSettings(armMotorConfig);
    m_armMotor.enableVoltageCompensation(true);
    m_armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setArmControlMode(ArmMode mode) {
    m_armMode = mode;
    this.resetArmAbsoluteRotation(0);
  }

  public void resetArmAbsoluteRotation(double rotations) {
    m_armMotor.setSelectedSensorPosition(rotations / kArmRotationsPerPulse);
  }

  public double getArmAbsoluteRotation() {
    return m_armMotor.getSelectedSensorPosition() * kArmRotationsPerPulse;
  }

  public double getArmAngleRadians() {
    return this.getArmAbsoluteRotation() * 2 * Math.PI;
  }

  public void setArmAngleRadians(double angle) {
    m_armAngleSetpointRadians = angle;
  }

  public double getArmTranslationMeters() {
    return this.getArmAbsoluteRotation() * kSprocketCircumferenceMeters;
  }

  @Config(name = "Arm Current PD Terms")
  private void setArmCurrentPDTerms(double kP, double kD) {
    m_armMotor.config_kP(0, kP);
    m_armMotor.config_kD(0, kD);
  }

  public void runArmControlLoop() {
    if (m_armMode == ArmMode.kRotation) {
      // Set arm
      double armFeedbackOutputCurrent =
          m_armControllerRotation.calculate(getArmAngleRadians(), m_armAngleSetpointRadians);

      double armAccelerationSetpoint =
          (m_armControllerRotation.getSetpoint().velocity - m_lastArmVelocitySetpointRotation)
              / Constants.kRIOLoopTime;
      double armGravityTorque =
          kGravityMetersPerSecondSquared * Math.sin(getArmAngleRadians()) / kArmLengthMeters;
      double armAccelerationTorque = armAccelerationSetpoint / kArmMomentOfIntertia;
      double armFeedforwardOutputCurrent =
          (1 / kFalconTorquePerAmp) * (armGravityTorque + armAccelerationTorque);

      m_armMotor.set(ControlMode.Current, armFeedforwardOutputCurrent + armFeedbackOutputCurrent);
    } else if (m_armMode == ArmMode.kTranslation) {
    }
  }

  @Override
  public void periodic() {
    this.runArmControlLoop();
  }
}
