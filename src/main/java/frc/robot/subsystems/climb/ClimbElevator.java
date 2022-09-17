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

  private final SR_ProfiledPIDController m_pidControllerUnloaded =
      new SR_ProfiledPIDController(
          kPElevatorUnloadedVoltsPerMeter,
          0,
          kDElevatorUnloadedVoltSecondsPerMeter,
          kElevatorControllerConstraintsUnloaded,
          kElevatorControlLoopTimeSeconds);

  private final SR_ProfiledPIDController m_pidControllerLoaded =
      new SR_ProfiledPIDController(
          kPElevatorLoadedVoltsPerMeter,
          0,
          kDElevatorLoadedVoltSecondsPerMeter,
          kElevatorControllerConstraintsLoaded,
          kElevatorControlLoopTimeSeconds);

  @Log(name = "last velocity setpoint")
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

  private boolean m_limitOverride = false;

  @Log(name = "position setpoint")
  private double m_positionSetpointMeters = 0;

  private final Servo m_ratchetServo = new Servo(kElevatorServoPort);

  @Log(name = "feedback output")
  private double m_feedbackOutput = 0;

  @Log(name = "feedforward output")
  private double m_feedforwardOutput = 0;

  private boolean m_hookedOnMidBar = false;

  private boolean m_voltageOverride = false;
  private double m_voltageSetpoint = 0;

  @Log(name = "output")
  double outputPercentage;

  @Log(name = "enabled")
  private boolean m_enabled = false;

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

    m_pidControllerUnloaded.setTolerance(
        kElevatorPositionToleranceMeters, kElevatorVelocityToleranceMetersPerSecond);
    m_pidControllerLoaded.setTolerance(
        kElevatorPositionToleranceMeters, kElevatorVelocityToleranceMetersPerSecond);

    m_loaded = false;
    this.resetPositionMeters(0);

    this.setRatchet(false);

    Shuffleboard.getTab("ClimbElevator").add("Controller Unloaded", m_pidControllerUnloaded);
    Shuffleboard.getTab("ClimbElevator").add("FF Unloaded", m_feedforwardUnloaded);

    Shuffleboard.getTab("ClimbElevator").add("Controller Loaded", m_pidControllerLoaded);
    Shuffleboard.getTab("ClimbElevator").add("FF Loaded", m_feedforwardLoaded);
  }

  @Log(name = "is inverted")
  public boolean isInverted() {
    return m_talon.getInverted();
  }

  public void invertTalon() {
    if (m_talon.getInverted()) {
      m_talon.setInverted(false);
    } else {
      m_talon.setInverted(true);
    }
  }

  @Log(name = "error")
  public double getError() {
    return Math.abs(m_positionSetpointMeters - this.getPositionMeters());
  }

  @Config(name = "Set elevator position")
  public void setPositionMeters(double position) {
    m_voltageOverride = false;
    m_positionSetpointMeters =
        MathUtil.clamp(position, kElevatorBottomStopPosition, kElevatorTopStopPosition);

    /// m_positionSetpointMeters = position;
  }

  public void setLimitOverride(boolean limitOverride) {
    m_limitOverride = limitOverride;
  }

  @Log(name = "Current elevator position")
  public double getPositionMeters() {
    return m_talon.getSelectedSensorPosition() * kSlideDistancePerPulseMeters;
  }

  @Config(name = "Reset elevator positon")
  public void resetPositionMeters(double position) {
    m_talon.setSelectedSensorPosition(position / kSlideDistancePerPulseMeters);
  }

  @Log(name = "Current elevator velocity")
  public double getVelocityMetersPerSecond() {
    return m_talon.getSelectedSensorVelocity() * kSlideDistancePerPulseMeters * 10;
  }

  @Log(name = "At position goal")
  private boolean atPositionGoal() {
    return Math.abs(this.getPositionMeters() - m_positionSetpointMeters)
        < kElevatorPositionToleranceMeters;
  }

  public void setHookedOnMidBar(boolean hooked) {
    m_hookedOnMidBar = hooked;
  }

  public boolean getHookedOnMidBar() {
    return m_hookedOnMidBar;
  }

  @Config(name = "set loaded")
  public void setMode(boolean loaded) {
    this.m_loaded = loaded;
  }

  // @Config(name = "Set voltage")
  public void setVoltage(double voltage) {
    m_voltageOverride = true;
    m_voltageSetpoint = voltage;
  }

  public void abort() {
    m_talon.set(ControlMode.Disabled, 0);
  }

  public void enable(boolean enabled) {
    m_enabled = enabled;
    if (enabled) {
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);
    } else {
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
      m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);
    }
  }

  public void runControlLoop() {
    if (m_enabled) {
      if (m_voltageOverride) {
        m_talon.set(ControlMode.PercentOutput, m_voltageSetpoint / Constants.kNominalVoltage);
      } else {
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

        outputPercentage =
            (feedbackOutputVoltage + feedforwardOutputVoltage) / Constants.kNominalVoltage;

        if (!m_limitOverride) {
          // outputPercentage = this.limitOnSlotSensors(outputPercentage);
        }

        m_feedbackOutput = feedbackOutputVoltage;
        m_feedforwardOutput = feedforwardOutputVoltage;
        // m_talon.set(ControlMode.PercentOutput, limitOnSlotSensors(outputPercentage));

        m_talon.set(ControlMode.PercentOutput, outputPercentage);

        if (m_loaded) {
          m_lastVelocitySetpoint = m_pidControllerLoaded.getSetpoint().velocity;
        } else {
          m_lastVelocitySetpoint = m_pidControllerUnloaded.getSetpoint().velocity;
        }
      }
    } else {
      m_talon.set(ControlMode.PercentOutput, 0);
    }
  }

  // public double limitOnSlotSensors(double voltage) {
  //   boolean topSlotSensorTripped = m_topSlotSensor.get();
  //   boolean bottomSlotSensorTripped = m_bottomSlotSensor.get();

  //   if (bottomSlotSensorTripped && voltage < 0) {
  //     return 0;
  //   } else if (topSlotSensorTripped && voltage > 0) {
  //     return 0;
  //   } else {
  //     return voltage;
  //   }
  // }

  @Log(name = "top tripped")
  public boolean topTripped() {
    return m_topSlotSensor.get();
  }

  @Log(name = "bottom tripped")
  public boolean bottomTripped() {
    return m_bottomSlotSensor.get();
  }

  public void updatePositionOnSlotSensors() {
    // boolean topSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());
    // // boolean bottomSlotSensorTripped = m_slotSensorDebounce.calculate(m_topSlotSensor.get());

    // // if (bottomSlotSensorTripped && this.getPositionMeters() >=
    // kElevatorSlotSensorBottomPosition)
    // // {
    // //   this.resetPositionMeters(kElevatorSlotSensorBottomPosition);
    // // } else
    // if (topSlotSensorTripped && this.getPositionMeters() <= kElevatorSlotSensorTopPosition) {
    //   this.resetPositionMeters(kElevatorSlotSensorTopPosition);
    // }
  }

  @Config.ToggleSwitch(name = "Set ratchet")
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
