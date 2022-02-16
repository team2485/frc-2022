package frc.robot.subsystems;

import static frc.robot.Constants.ClimbElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class ClimbElevator extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kElevatorTalonPort);
  private final DigitalInput m_topSlotSensor = new DigitalInput(kElevatorSlotSensorTopPort);
  private final DigitalInput m_bottomSlotSensor = new DigitalInput(kElevatorSlotSensorBottomPort);

  private final Debouncer m_slotSensorDebounce =
      new Debouncer(kSlotSensorDebounceTime, DebounceType.kBoth);

  private final ProfiledPIDController m_pidController =
      new ProfiledPIDController(kPElevator, 0, kDElevator, kElevatorControllerConstraints);

  private double m_lastVelocitySetpoint = 0;

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          ksElevatorVolts, kvElevatorVoltSecondsPerMeter, kaElevatorVoltSecondsSquaredPerMeter);

  private double m_positionSetpointMeters = 0;

  public ClimbElevator() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true,
            kElevatorSupplyCurrentLimitAmps,
            kElevatorSupplyCurrentThresholdAmps,
            kElevatorSupplyCurrentThresholdTimeSecs);
    talonConfig.statorCurrLimit =
        new StatorCurrentLimitConfiguration(
            true,
            kElevatorStatorCurrentLimitAmps,
            kElevatorStatorCurrentThresholdAmps,
            kElevatorStatorCurrentThresholdTimeSecs);

    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);
  }

  public void setElevatorPositionMeters(double position) {
    m_positionSetpointMeters = position;
  }

  public double getElevatorPositionMeters() {
    return m_talon.getSelectedSensorPosition() * kSlideDistancePerPulseMeters;
  }

  public void resetElevatorPositionMeters(double position) {
    m_talon.setSelectedSensorPosition(position / kSlideDistancePerPulseMeters);
  }

  public double getElevatorVelocityMetersPerSecond() {
    return m_talon.getSelectedSensorVelocity() * kSlideDistancePerPulseMeters * 0.1;
  }

  public void runSlideControlLoop() {
    // Set slider
    double feedbackOutputVoltage =
        m_pidController.calculate(m_positionSetpointMeters, this.getElevatorPositionMeters());

    double feedforwardOutputVoltage =
        m_feedforward.calculate(
            m_lastVelocitySetpoint, m_pidController.getSetpoint().velocity, Constants.kRIOLoopTime);

    double outputPercentage =
        (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;

    m_talon.set(ControlMode.PercentOutput, limitOnSlotSensors(outputPercentage));
    m_lastVelocitySetpoint = m_pidController.getSetpoint().velocity;
  }

  public double limitOnSlotSensors(double voltage) {
    boolean topSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());
    boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());

    if (bottomSlotSensorTripped && voltage < 0) {
      return 0;
    } else if (topSlotSensorTripped && voltage > 0) {
      return 0;
    } else {
      return voltage;
    }
  }

  public void updatePositionOnSlotSensors() {
    boolean topSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());
    boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());

    if (bottomSlotSensorTripped
        && this.getElevatorPositionMeters() >= kElevatorSlotSensorBottomPosition) {
      this.resetElevatorPositionMeters(kElevatorSlotSensorBottomPosition);
    } else if (topSlotSensorTripped
        && this.getElevatorPositionMeters() <= kElevatorSlotSensorTopPosition) {
      this.resetElevatorPositionMeters(kElevatorSlotSensorTopPosition);
    }
  }

  @Override
  public void periodic() {
    this.updatePositionOnSlotSensors();
    this.runSlideControlLoop();
  }
}
