package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbArmConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_ElevatorFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class ClimbArm extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kArmTalonPort);

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  // input of position error, output of voltage
  private final SR_ProfiledPIDController m_pidControllerTranslation =
      new SR_ProfiledPIDController(
          kPArmTranslationVoltsPerMeter,
          0,
          kDArmTranslationVoltSecondsPerMeter,
          kArmControllerConstraintsTranslation);

  // input of acceleration, output of voltage
  private final SR_ElevatorFeedforward m_feedforwardTranslation =
      new SR_ElevatorFeedforward(
          ksArmTranslationVolts,
          kgArmTranslationVolts,
          kvArmTranslationVoltSecondsPerMeter,
          kaArmTranslationVoltSecondsSquaredPerMeter);

  private double m_translationSetpointMeters = 0;
  private double m_lastVelocitySetpointTranslation = 0;

  public ClimbArm() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kArmSupplyCurrentLimitAmps,
            kArmSupplyCurrentThresholdAmps,
            kArmSupplyCurrentThresholdTimeSecs);
    talonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kArmStatorCurrentLimitAmps,
            kArmStatorCurrentThresholdAmps,
            kArmStatorCurrentThresholdTimeSecs);

    // current pi and feedforward terms
    talonConfig.slot0.kP = kPArmCurrentOutputUnitsPerMilliamp;
    talonConfig.slot0.kI = kIArmCurrentOutputUnitsPerMilliamp;
    // talonConfig.slot0.kF = kVArmCurrentOutputUnitsPerMilliamp;
    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);

    m_pidControllerTranslation.setTolerance(kArmTranslationToleranceMeters);

    this.resetAbsoluteRotation(0);

    Shuffleboard.getTab("ClimbArm").add("Controller Translation", m_pidControllerTranslation);
    Shuffleboard.getTab("ClimbArm").add("FF Translation", m_feedforwardTranslation);
  }

  @Config(name = "Reset absolute rotation")
  public void resetAbsoluteRotation(double rotations) {
    m_talon.setSelectedSensorPosition(rotations / kArmRotationsPerPulse);
  }

  @Log(name = "Current absolute rotation")
  public double getAbsoluteRotation() {
    return m_talon.getSelectedSensorPosition() * kArmRotationsPerPulse;
  }

  public double getTranslationMeters() {
    return this.getAbsoluteRotation() * kSprocketCircumferenceMeters;
  }

  public void setTranslationMeters(double translation) {
    m_voltageOverride = false;
    m_translationSetpointMeters = translation;
  }

  // @Config(name = "set current (amps)")
  // private void setCurrent(double amps) {
  //   m_talon.set(ControlMode.Current, amps * 1000);
  // }

  @Log(name = "Stator current (amps)")
  public double getStatorCurrentAmps() {
    return m_talon.getStatorCurrent();
  }

  @Log(name = "Supply current (amps)")
  public double getSupplyCurrentAmps() {
    return m_talon.getSupplyCurrent();
  }

  @Config(name = "Set voltage")
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public boolean atPositionGoal() {
    return m_pidControllerTranslation.atGoal();
  }

  public void runControlLoop() {

    if (m_voltageOverride) {
      m_talon.set(ControlMode.PercentOutput, m_voltageSetpoint / Constants.kNominalVoltage);
    } else {
      double feedbackOutputVoltage =
          m_pidControllerTranslation.calculate(getTranslationMeters(), m_translationSetpointMeters);

      double feedforwardOutputVoltage =
          m_feedforwardTranslation.calculate(
              m_lastVelocitySetpointTranslation,
              m_pidControllerTranslation.getSetpoint().velocity,
              kArmControlLoopTimeSeconds);

      double outputPercentage =
          (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;

      m_talon.set(ControlMode.PercentOutput, outputPercentage);

      m_lastVelocitySetpointTranslation = m_pidControllerTranslation.getSetpoint().velocity;
    }
  }

  @Override
  public void periodic() {}
}
