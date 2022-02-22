package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbElevatorConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_ElevatorFeedforward;
import frc.team2485.WarlordsLib.sendableRichness.SR_ProfiledPIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class ClimbElevator extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kElevatorTalonPort);

  @Log(name = "Top Slot Sensor")
  private final DigitalInput m_topSlotSensor = new DigitalInput(kElevatorSlotSensorTopPort);

  @Log(name = "Bottom Slot Sensor")
  private final DigitalInput m_bottomSlotSensor = new DigitalInput(kElevatorSlotSensorBottomPort);

  private final Debouncer m_slotSensorDebounce =
      new Debouncer(kSlotSensorDebounceTime, DebounceType.kBoth);
  private final SR_ProfiledPIDController m_pidControllerUnloaded =
      new SR_ProfiledPIDController(
          kPElevatorUnloadedVoltsPerMeter,
          0,
          kDElevatorUnloadedVoltSecondsPerMeter,
          kElevatorControllerConstraintsUnloaded);

  private final SR_ProfiledPIDController m_pidControllerLoaded =
      new SR_ProfiledPIDController(
          kPElevatorLoadedVoltsPerMeter,
          0,
          kDElevatorLoadedVoltSecondsPerMeter,
          kElevatorControllerConstraintsLoaded);

  private double m_lastVelocitySetpoint = 0;

  // @Log(name = "Elevator feedforward unloaded")
  private final SR_ElevatorFeedforward m_feedforwardUnloaded =
      new SR_ElevatorFeedforward(
          ksElevatorUnloadedVolts,
          kgElevatorUnloadedVolts,
          kvElevatorUnloadedVoltSecondsPerMeter,
          kaElevatorUnloadedVoltSecondsSquaredPerMeter);

  // @Log(name = "Elevator feedforward loaded")
  private final SR_ElevatorFeedforward m_feedforwardLoaded =
      new SR_ElevatorFeedforward(
          ksElevatorLoadedVolts,
          kgElevatorLoadedVolts,
          kvElevatorLoadedVoltSecondsPerMeter,
          kaElevatorLoadedVoltSecondsSquaredPerMeter);

  @Log(name = "loaded")
  private boolean m_loaded; // unloaded, true is loaded

  private boolean m_zeroOverride = false;

  @Log(name = "position setpoint")
  private double m_positionSetpointMeters = 0;

  private final Servo m_ratchetServo = new Servo(kElevatorServoPort);

  @Log(name = "feedback output")
  private double m_feedbackOutput = 0;

  @Log(name = "feedforward output")
  private double m_feedforwardOutput = 0;

  private boolean m_hookedOnMidBar = false;

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
    m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);

    m_talon.setInverted(true);

    m_pidControllerUnloaded.setTolerance(kElevatorPositionToleranceMeters);
    m_pidControllerLoaded.setTolerance(kElevatorPositionToleranceMeters);

    m_loaded = false;
    this.resetPositionMeters(0);

    this.setRatchet(true);

    Shuffleboard.getTab("ClimbElevator").add("Controller Unloaded", m_pidControllerUnloaded);
    Shuffleboard.getTab("ClimbElevator").add("FF Unloaded", m_feedforwardUnloaded);

    Shuffleboard.getTab("ClimbElevator").add("Controller Loaded", m_pidControllerLoaded);
    Shuffleboard.getTab("ClimbElevator").add("FF Loaded", m_feedforwardLoaded);
  }

  @Config(name = "Set elevator position")
  public void setPositionMeters(double position) {
    m_zeroOverride = false;
    m_positionSetpointMeters =
        MathUtil.clamp(position, kElevatorBottomStopPosition, kElevatorTopStopPosition);

    /// m_positionSetpointMeters = position;
  }

  @Log(name = "Current elevator position")
  public double getPositionMeters() {
    return m_talon.getSelectedSensorPosition() * kSlideDistancePerPulseMeters;
  }

  @Config(name = "Reset elevator position")
  public void resetPositionMeters(double position) {
    m_talon.setSelectedSensorPosition(position / kSlideDistancePerPulseMeters);
  }

  @Log(name = "Current elevator velocity")
  public double getVelocityMetersPerSecond() {
    return m_talon.getSelectedSensorVelocity() * kSlideDistancePerPulseMeters * 0.1;
  }

  @Log(name = "At position goal")
  private boolean atPositionGoal() {
    if (m_loaded) {
      return m_pidControllerLoaded.atGoal();
    } else {
      return m_pidControllerUnloaded.atGoal();
    }
  }

  public void setHookedOnMidBar(boolean hooked) {
    m_hookedOnMidBar = hooked;
  }

  public boolean getHookedOnMidBar() {
    return m_hookedOnMidBar;
  }

  public void setVoltage(double voltage) {
    m_talon.set(ControlMode.PercentOutput, voltage / Constants.kNominalVoltage);
  }

  public void zeroOverride() {
    m_zeroOverride = true;
  }

  public void setMode(boolean loaded) {
    this.m_loaded = loaded;
  }

  public void runControlLoop() {
    double feedbackOutputVoltage = 0;

    if (m_loaded) {
      feedbackOutputVoltage =
          m_pidControllerLoaded.calculate(this.getPositionMeters(), m_positionSetpointMeters);
    } else {
      feedbackOutputVoltage =
          m_pidControllerUnloaded.calculate(this.getPositionMeters(), m_positionSetpointMeters);
    }

    double feedforwardOutputVoltage = 0;
    if (m_loaded) {
      feedforwardOutputVoltage =
          m_feedforwardLoaded.calculate(
              m_lastVelocitySetpoint,
              m_pidControllerLoaded.getSetpoint().velocity,
              kElevatorControlLoopTimeSeconds);
    } else {
      feedforwardOutputVoltage =
          m_feedforwardUnloaded.calculate(
              m_lastVelocitySetpoint,
              m_pidControllerUnloaded.getSetpoint().velocity,
              kElevatorControlLoopTimeSeconds);
    }

    double outputPercentage =
        (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;

    m_feedbackOutput = feedbackOutputVoltage;
    m_feedforwardOutput = feedforwardOutputVoltage;
    // m_talon.set(ControlMode.PercentOutput, limitOnSlotSensors(outputPercentage));

    if (m_zeroOverride) {
      m_talon.set(ControlMode.PercentOutput, 0);
    } else {
      m_talon.set(ControlMode.PercentOutput, outputPercentage);
    }

    if (m_loaded) {
      m_lastVelocitySetpoint = m_pidControllerLoaded.getSetpoint().velocity;
    } else {
      m_lastVelocitySetpoint = m_pidControllerUnloaded.getSetpoint().velocity;
    }
  }

  public double limitOnSlotSensors(double voltage) {
    // boolean topSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());
    boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_bottomSlotSensor.get());

    if (bottomSlotSensorTripped && voltage < 0) {
      return 0;
      // } else
      // if (topSlotSensorTripped && voltage > 0) {
      //   return 0;
    } else {
      return voltage;
    }
  }

  public void updatePositionOnSlotSensors() {
    boolean topSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());
    // boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());

    // if (bottomSlotSensorTripped && this.getPositionMeters() >= kElevatorSlotSensorBottomPosition)
    // {
    //   this.resetPositionMeters(kElevatorSlotSensorBottomPosition);
    // } else
    if (topSlotSensorTripped && this.getPositionMeters() <= kElevatorSlotSensorTopPosition) {
      this.resetPositionMeters(kElevatorSlotSensorTopPosition);
    }
  }

  @Config(name = "Set Ratchet")
  public void setRatchet(boolean engaged) {
    if (engaged) {
      m_ratchetServo.set(kElevatorServoEngageValue);
    } else {
      m_ratchetServo.set(kElevatorServoDisengageValue);
    }
  }

  @Override
  public void periodic() {
    // m_talon.setVoltage(-0.75);
    // this.updatePositionOnSlotSensors();
    // this.runControlLoop();
    // remove after move to commands
    // this.setRatchet(false);
  }
}
