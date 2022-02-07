package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Climber extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_slideMotor = new WPI_TalonFX(kSlideTalonPort);
  private final Encoder m_slideSlotSensor =
      new Encoder(kSlideSlotSensorPort1, kSlideSlotSensorPort2);

  private final ProfiledPIDController m_slideController =
      new ProfiledPIDController(kPSlide, 0, kDSlide, kSlideControllerConstraints);

  private double m_lastSlideVelocitySetpoint = 0;

  private final SimpleMotorFeedforward m_slideFeedForward =
      new SimpleMotorFeedforward(
          ksSlideVolts, kvSlideVoltSecondsPerMeter, kaSlideVoltSecondsSquaredPerMeter);

  private double m_slidePositionSetpointMeters = 0;

  private final WPI_TalonFX m_armMotor = new WPI_TalonFX(kArmTalonPort);

  public enum ArmMode {
    kTranslation,
    kRotation
  }

  private double m_armAngleRadians = 0; // from vertical down
  private double m_armAngleSetpointRadians = 0;
  private double m_lastArmVelocitySetpointRotation = 0;

  private final ProfiledPIDController m_armControllerRotation =
      new ProfiledPIDController(kPArmRotation, 0, kDArmRotation, kArmControllerConstraintsRotation);

  public Climber() {
    TalonFXConfiguration slideMotorConfig = new TalonFXConfiguration();
    slideMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    slideMotorConfig.supplyCurrLimit.currentLimit = kSlideCurrentLimitAmps;
    slideMotorConfig.supplyCurrLimit.enable = true;
    m_slideMotor.configAllSettings(slideMotorConfig);
    m_slideMotor.enableVoltageCompensation(true);
    m_slideMotor.setNeutralMode(NeutralMode.Brake);

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    armMotorConfig.supplyCurrLimit.currentLimit = kSlideCurrentLimitAmps;
    armMotorConfig.supplyCurrLimit.enable = true;
    armMotorConfig.slot0.kP = kPArmCurrent;
    armMotorConfig.slot0.kD = kDArmCurrent;
    m_slideMotor.configAllSettings(slideMotorConfig);
    m_slideMotor.enableVoltageCompensation(true);
    m_slideMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setSlidePositionMeters(double position) {
    m_slidePositionSetpointMeters = position;
  }

  public double getSlidePositionMeters() {
    return m_slideSlotSensor.getDistance();
  }

  public void resetArmAbsoluteRotation(double rotationRadians) {
    m_armMotor.setSelectedSensorPosition(rotationRadians / kArmRadiansPerPulse);
  }

  public double getArmAbsoluteRotationRadians() {
    return m_armMotor.getSelectedSensorPosition() * kArmRadiansPerPulse;
  }

  public double getArmAngleRadians() {
    return getArmAbsoluteRotationRadians();
  }

  public void setArmAngleRadians(double angle) {
    m_armAngleSetpointRadians = angle;
  }

  @Config(name = "Arm Current PD Terms")
  private void setArmCurrentPDTerms(double kP, double kD) {
    m_armMotor.config_kP(0, kP);
    m_armMotor.config_kD(0, kD);
  }

  public void periodic() {

    // Set slider
    double slideControllerOutput =
        m_slideController.calculate(m_slidePositionSetpointMeters, this.getSlidePositionMeters());

    double slideFeedforwardOutputVoltage =
        m_slideFeedForward.calculate(
            m_lastSlideVelocitySetpoint,
            m_slideController.getSetpoint().velocity,
            Constants.kRIOLoopTime);
    m_slideMotor.set(
        ControlMode.PercentOutput,
        (slideControllerOutput + slideFeedforwardOutputVoltage) * Constants.kNominalVoltage);

    m_lastSlideVelocitySetpoint = m_slideController.getSetpoint().velocity;

    // Set arm

    double armControllerOutputCurrent =
        m_armControllerRotation.calculate(getArmAngleRadians(), m_armAngleSetpointRadians);

    double armAccelerationSetpoint =
        (m_armControllerRotation.getSetpoint().velocity - m_lastArmVelocitySetpointRotation)
            / Constants.kRIOLoopTime;
    double armGravityTorque =
        kGravityMetersPerSecondSquared * Math.sin(getArmAngleRadians()) / kArmLengthMeters;
    double armAccelerationTorque = armAccelerationSetpoint / kArmMomentOfIntertia;
    double armFeedforwardOutputCurrent =
        (1 / kFalconTorquePerAmp) * (armGravityTorque + armAccelerationTorque);

    m_armMotor.set(ControlMode.Current, armFeedforwardOutputCurrent + armControllerOutputCurrent);
  }
}
