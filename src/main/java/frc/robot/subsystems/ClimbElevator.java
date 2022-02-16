package frc.robot.subsystems;

import static frc.robot.Constants.ClimbElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class ClimbElevator extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kElevatorTalonPort);
  private final DigitalInput m_topSlotSensor = new DigitalInput(kElevatorSlotSensorTopPort);
  private final DigitalInput m_bottomSlotSensor = new DigitalInput(kElevatorSlotSensorBottomPort);

  private final Debouncer m_slotSensorDebounce =
      new Debouncer(kSlotSensorDebounceTime, DebounceType.kBoth);

  @Config(name = "Elevator controller")
  private final ProfiledPIDController m_pidController =
      new ProfiledPIDController(kPElevator, 0, kDElevator, kElevatorControllerConstraints);

  private double m_lastVelocitySetpoint = 0;

  @Log(name = "Elevator feedforward")
  private final SR_SimpleMotorFeedforward m_feedforward =
      new SR_SimpleMotorFeedforward(
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

  @Config(name = "Set elevator position")
  public void setPositionMeters(double position) {
    m_positionSetpointMeters = position;
  }

  @Log(name = "Current elevator position")
  public double getPositionMeters() {
    return m_talon.getSelectedSensorPosition() * kSlideDistancePerPulseMeters;
  }

  @Config(name = "Reset elevator position")
  public void resetPositionMeters(double position) {
    m_talon.setSelectedSensorPosition(position / kSlideDistancePerPulseMeters);
  }

  @Log(name = "Current elevator veloity")
  public double getVelocityMetersPerSecond() {
    return m_talon.getSelectedSensorVelocity() * kSlideDistancePerPulseMeters * 0.1;
  }

  public void runControlLoop() {
    double feedbackOutputVoltage =
        m_pidController.calculate(m_positionSetpointMeters, this.getPositionMeters());

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
    boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_bottomSlotSensor.get());

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

    if (bottomSlotSensorTripped && this.getPositionMeters() >= kElevatorSlotSensorBottomPosition) {
      this.resetPositionMeters(kElevatorSlotSensorBottomPosition);
    } else if (topSlotSensorTripped && this.getPositionMeters() <= kElevatorSlotSensorTopPosition) {
      this.resetPositionMeters(kElevatorSlotSensorTopPosition);
    }
  }

  @Override
  public void periodic() {
    this.updatePositionOnSlotSensors();
    this.runControlLoop();
  }
}
