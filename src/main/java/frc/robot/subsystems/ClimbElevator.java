package frc.robot.subsystems;

import static frc.robot.Constants.ClimbElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private final WPI_TalonFX m_elevatorMotor = new WPI_TalonFX(kElevatorTalonPort);
  private final DigitalInput m_elevatorSlotSensor = new DigitalInput(kElevatorSlotSensorPort);
  private final Debouncer m_elevatorSlotSensorDebounce =
      new Debouncer(kSlotSensorDebounceTime, DebounceType.kBoth);

  private final ProfiledPIDController m_elevatorController =
      new ProfiledPIDController(kPElevator, 0, kDElevator, kElevatorControllerConstraints);

  private double m_lastElevatorVelocitySetpoint = 0;

  private final SimpleMotorFeedforward m_elevatorFeedForward =
      new SimpleMotorFeedforward(
          ksElevatorVolts, kvElevatorVoltSecondsPerMeter, kaElevatorVoltSecondsSquaredPerMeter);

  private double m_elevatorPositionSetpointMeters = 0;

  private boolean m_elevatorDirection = false; // true is top, false is bottom

  public ClimbElevator() {
    TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
    elevatorMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    elevatorMotorConfig.supplyCurrLimit.currentLimit = kElevatorCurrentLimitAmps;
    elevatorMotorConfig.supplyCurrLimit.enable = true;
    m_elevatorMotor.configAllSettings(elevatorMotorConfig);
    m_elevatorMotor.enableVoltageCompensation(true);
    m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setElevatorPositionMeters(double position) {
    m_elevatorPositionSetpointMeters = position;
    if (position > this.getElevatorPositionMeters()) {
      m_elevatorDirection = true;
    } else {
      m_elevatorDirection = false;
    }
  }

  public double getElevatorPositionMeters() {
    return m_elevatorMotor.getSelectedSensorPosition() * kSlideDistancePerPulseMeters;
  }

  public void resetElevatorPositionMeters(double position) {
    m_elevatorMotor.setSelectedSensorPosition(position / kSlideDistancePerPulseMeters);
  }

  public double getElevatorVelocityMetersPerSecond() {
    return m_elevatorMotor.getSelectedSensorVelocity() * kSlideDistancePerPulseMeters * 0.1;
  }

  public void runSlideControlLoop() {
    // Set slider
    double slideFeedbackOutputVoltage =
        m_elevatorController.calculate(
            m_elevatorPositionSetpointMeters, this.getElevatorPositionMeters());

    double slideFeedforwardOutputVoltage =
        m_elevatorFeedForward.calculate(
            m_lastElevatorVelocitySetpoint,
            m_elevatorController.getSetpoint().velocity,
            Constants.kRIOLoopTime);
    m_elevatorMotor.set(
        ControlMode.PercentOutput,
        (slideFeedbackOutputVoltage + slideFeedforwardOutputVoltage) * Constants.kNominalVoltage);

    m_lastElevatorVelocitySetpoint = m_elevatorController.getSetpoint().velocity;
  }

  public void resetOnSlotSensor() {
    boolean slotSensorTripped = m_elevatorSlotSensorDebounce.calculate(m_elevatorSlotSensor.get());

    if (slotSensorTripped) {
      if (m_elevatorDirection == true) {
        this.resetElevatorPositionMeters(kElevatorSlotSensorTopPosition);
        m_elevatorMotor.set(0);
      } else {
        this.resetElevatorPositionMeters(kElevatorSlotSensorTopPosition);
        m_elevatorMotor.set(0);
      }
    }
  }

  @Override
  public void periodic() {
    this.resetOnSlotSensor();
    this.runSlideControlLoop();
  }
}
