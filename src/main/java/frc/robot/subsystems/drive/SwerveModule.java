package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule implements Loggable {

  public final WL_TalonFX m_driveMotor;

  private final SR_SimpleMotorFeedforward m_driveFeedforward =
      new SR_SimpleMotorFeedforward(
          ksDriveVolts, kvDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter);

  public final WL_TalonFX m_turningMotor;
  private final CANCoder m_turningEncoder;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  private final String m_moduleID;

  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      Rotation2d zero,
      boolean driveInverted,
      String moduleID) {
    this.m_moduleID = moduleID;

    // Drive motor configuration
    // -- voltage compensation, current limiting, P term, brake mode
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    driveMotorConfig.supplyCurrLimit.currentLimit = kDriveSupplyCurrentLimitAmps;
    driveMotorConfig.supplyCurrLimit.enable = true;
    driveMotorConfig.statorCurrLimit.currentLimit = kDriveStatorCurrentLimitAmps;
    driveMotorConfig.statorCurrLimit.triggerThresholdCurrent = kDriveStatorCurrentLimitAmps;
    driveMotorConfig.statorCurrLimit.triggerThresholdTime = kDriveStatorCurrentThresholdTimeSecs;
    driveMotorConfig.slot0.kP = kPDrive;
    driveMotorConfig.slot0.allowableClosedloopError = 0.01 / kDriveDistMetersPerPulse / 10;
    driveMotorConfig.velocityMeasurementWindow = 1;
    driveMotorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    this.m_driveMotor = new WL_TalonFX(driveMotorID);
    m_driveMotor.configAllSettings(driveMotorConfig);
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.setInverted(driveInverted);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);
    // Turning motor configuration
    // -- voltage compensation, current limiting, P D F terms, motion magic, brake mode
    TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();
    turningMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    turningMotorConfig.supplyCurrLimit.currentLimit = kTurningSupplyCurrentLimitAmps;
    turningMotorConfig.supplyCurrLimit.enable = true;
    turningMotorConfig.statorCurrLimit.currentLimit = kTurningStatorCurrentLimitAmps;
    turningMotorConfig.statorCurrLimit.triggerThresholdCurrent = kTurningStatorCurrentLimitAmps;
    turningMotorConfig.statorCurrLimit.triggerThresholdTime =
        kTurningStatorCurrentThresholdTimeSecs;
    turningMotorConfig.slot0.kP = kPTurningOutputUnit100MsPerSensorUnit;
    turningMotorConfig.slot0.kD = kDTurningOutputUnit100MsSquaredPerSensorUnit;
    turningMotorConfig.slot0.kF = kFTurningOutputUnit100MsPerSensorUnit;
    turningMotorConfig.motionCruiseVelocity = kModuleMaxSpeedTurningPulsesPer100Ms;
    turningMotorConfig.motionAcceleration = kModuleMaxAccelerationTurningPulsesPer100MsSquared;
    this.m_turningMotor = new WL_TalonFX(turningMotorID);
    m_turningMotor.configAllSettings(turningMotorConfig);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
    m_turningMotor.configIntegratedSensorAbsoluteRange(
        AbsoluteSensorRange.Signed_PlusMinus180, Constants.kCANTimeoutMs);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configAllowableClosedloopError(
        0, kTurningPositionToleranceSensorUnits, Constants.kCANTimeoutMs);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    // Turning encoder configuration
    // -- configures offset and sensor range to +-180
    CANCoderConfiguration turningEncoderConfig = new CANCoderConfiguration();
    turningEncoderConfig.magnetOffsetDegrees = -zero.getDegrees();
    turningEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    turningEncoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    m_turningEncoder = new CANCoder(turningEncoderID);
    m_turningEncoder.configAllSettings(turningEncoderConfig);
    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

    try {
      Thread.sleep(255);
    } catch (Exception e) {
      e.printStackTrace();
    }
    double absolutePosition = m_turningEncoder.getAbsolutePosition();
    m_turningMotor.setSelectedSensorPosition(
        Units.degreesToRadians(absolutePosition) / kTurningRadiansPerPulse, 0, 1000);
  }

  /**
   * Optimizes swerve module state, then sets motors to it.
   *
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean inverted) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getIntegratedHeading());
    this.setSpeedMetersPerSecond(
        inverted ? -state.speedMetersPerSecond : state.speedMetersPerSecond);
    this.setHeading(state.angle);
    m_desiredState = state;
  }

  /**
   * Returns current swerve module state.
   *
   * @return current state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeedMetersPerSecond(), this.getIntegratedHeading());
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Log(name = "Desired heading degrees")
  public double getDesiredHeadingDegrees() {
    return m_desiredState.angle.getDegrees();
  }

  @Log(name = "Desired speed m/s")
  public double getDesiredSpeedMetersPerSecond() {
    return m_desiredState.speedMetersPerSecond;
  }

  /**
   * Returns current heading.
   *
   * @return current heading in degrees
   */
  @Log(name = "current rotation (integrated)")
  private double getIntegratedHeadingDegrees() {
    return getIntegratedHeading().getDegrees();
  }

  /**
   * Returns current heading.
   *
   * @return current heading in degrees
   */
  private double getHeadingSetpointDegrees() {
    return Units.radiansToDegrees(m_turningMotor.getClosedLoopTarget() * kTurningRadiansPerPulse);
  }

  /**
   * Returns current heading.
   *
   * @return heading as Rotation2d
   */
  private Rotation2d getIntegratedHeading() {
    return new Rotation2d(m_turningMotor.getSelectedSensorPosition() * kTurningRadiansPerPulse);
  }

  // @Log(name = "Integrated sensor Pulses")
  private double getIntegratedSensorPosition() {
    return m_turningMotor.getSelectedSensorPosition();
  }

  // @Log(name = "Can coder heading degreees")
  private double getCANCoderHeadingDegrees() {
    return getCANCoderHeading().getDegrees();
  }

  private Rotation2d getCANCoderHeading() {
    return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
  }

  /**
   * Optimizes and then sets heading, intended for Shuffleboard use.
   *
   * @param desiredRotationDegrees desired heading in degrees
   */
  // @Config(name = "set rotation")
  public void setHeadingDegrees(double desiredRotationDegrees) {
    Rotation2d currentRotation = this.getIntegratedHeading();
    // use WPILib's swervemodulestate optimization to minimize change in heading
    Rotation2d optimizedDesiredRotation =
        SwerveModuleState.optimize(
                new SwerveModuleState(0, Rotation2d.fromDegrees(desiredRotationDegrees)),
                currentRotation)
            .angle;
    this.setHeading(optimizedDesiredRotation);
  }

  /**
   * Sets already-optimized heading, using motion-profiled on-motor PID control
   *
   * @param optimizedDesiredRotation desired headiing as Rotation2d
   */
  public void setHeading(Rotation2d optimizedDesiredRotation) {
    double referencePulses = optimizedDesiredRotation.getRadians() / kTurningRadiansPerPulse;

    m_turningMotor.set(ControlMode.MotionMagic, referencePulses);
  }

  @Log(name = "Drive motor temperature")
  public double getDriveMotorTemperatureCelsius() {
    return m_driveMotor.getTemperature();
  }

  @Log(name = "Turning motor temperature")
  public double getTurningMotorTemperatureCelsius() {
    return m_turningMotor.getTemperature();
  }

  /**
   * Returns current speed.
   *
   * @return speed in meters per second
   */
  @Log(name = "Speed meters per second")
  private double getSpeedMetersPerSecond() {
    return m_driveMotor.getSelectedSensorVelocity() * kDriveDistMetersPerPulse * 10;
  }

  /**
   * Sets drive motor, using on-motor velocity PID and on-RIO feedforward
   *
   * @param desiredSpeed desired speed in meters per second
   */
  // @Config(name = "Set speed meters per second")
  public void setSpeedMetersPerSecond(double desiredSpeed) {
    m_driveMotor.set(
        ControlMode.Velocity,
        desiredSpeed / kDriveDistMetersPerPulse * 0.1,
        DemandType.ArbitraryFeedForward,
        m_driveFeedforward.calculate(desiredSpeed) / Constants.kNominalVoltage);
  }

  /** Resets drive encoder. */
  public void resetDriveEncoder() {
    m_driveMotor.setSelectedSensorPosition(0);
  }

  /**
   * Configures behavior of drive motor on 0 input.
   *
   * @param mode NeutralMode.Brake or NeutralMode.Coast
   */
  public void setDriveNeutralMode(NeutralMode mode) {
    m_driveMotor.setNeutralMode(mode);
  }

  /**
   * Configures behavior of turning motor on 0 input.
   *
   * @param mode NeutralMode.Brake or NeutralMode.Coast
   */
  public void setTurningNeutralMode(NeutralMode mode) {
    m_turningMotor.setNeutralMode(mode);
  }

  public static double convertToSteeringRange(double angleRadians) {

    double value = (angleRadians) % (2 * Math.PI);
    if (value > Math.PI) {
      value -= 2 * Math.PI;
    }
    return value;
  }

  @Override
  public String configureLogName() {
    return m_moduleID;
  }

  @Override
  public LayoutType configureLayoutType() {
    return BuiltInLayouts.kGrid;
  }

  @Override
  public int[] configureLayoutSize() {
    int[] size = {3, 4};
    return size;
  }
}
