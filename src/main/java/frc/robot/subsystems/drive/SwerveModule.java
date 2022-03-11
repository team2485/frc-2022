package frc.robot.subsystems.drive;

import static frc.robot.Constants.ModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule implements Loggable {

  public final WL_TalonFX m_driveMotor;

  private final SR_SimpleMotorFeedforward m_driveFeedforward =
      new SR_SimpleMotorFeedforward(
          ksDriveVolts, kvDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter);

  public final WL_TalonFX m_turningMotor;
  private final CANCoder m_turningEncoder;

  private final String m_moduleID;

  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      Rotation2d zero,
      String moduleID) {
    this.m_moduleID = moduleID;

    // Drive motor configuration
    // -- voltage compensation, current limiting, P term, brake mode
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    driveMotorConfig.supplyCurrLimit.currentLimit = kDriveCurrentLimitAmps;
    driveMotorConfig.supplyCurrLimit.enable = true;
    driveMotorConfig.slot0.kP = kPDrive;
    this.m_driveMotor = new WL_TalonFX(driveMotorID);
    m_driveMotor.configAllSettings(driveMotorConfig);
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.setStatusFramePeriod(1, 255);
    m_driveMotor.setStatusFramePeriod(2, 255);
    m_driveMotor.setInverted(true);

    // Turning motor configuration
    // -- voltage compensation, current limiting, P D F terms, motion magic, brake mode
    TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();
    turningMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
    turningMotorConfig.supplyCurrLimit.currentLimit = kTurningCurrentLimitAmps;
    turningMotorConfig.supplyCurrLimit.enable = true;
    turningMotorConfig.slot0.kP = kPTurning;
    turningMotorConfig.slot0.kD = kDTurning;
    turningMotorConfig.slot0.kF = kFTurning;
    turningMotorConfig.motionCruiseVelocity = kModuleMaxSpeedTurningPulsesPer100Ms;
    turningMotorConfig.motionAcceleration = kModuleMaxAccelerationTurningPulsesPer100MsSquared;
    this.m_turningMotor = new WL_TalonFX(turningMotorID);
    m_turningMotor.configAllSettings(turningMotorConfig);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setStatusFramePeriod(1, 255);
    m_turningMotor.setStatusFramePeriod(2, 255);

    // Turning encoder configuration
    // -- configures offset and sensor range to +-180
    CANCoderConfiguration turningEncoderConfig = new CANCoderConfiguration();
    turningEncoderConfig.magnetOffsetDegrees = -zero.getDegrees();
    turningEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    turningEncoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    m_turningEncoder = new CANCoder(turningEncoderID);
    m_turningEncoder.configAllSettings(turningEncoderConfig);
  }

  /**
   * Optimizes swerve module state, then sets motors to it.
   *
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getHeading());
    this.setSpeedMetersPerSecond(state.speedMetersPerSecond);
    this.setHeading(state.angle);
  }

  /**
   * Returns current swerve module state.
   *
   * @return current state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getSpeedMetersPerSecond(), this.getHeading());
  }

  /**
   * Returns current heading.
   *
   * @return current heading in degrees
   */
  @Log(name = "current rotation")
  private double getHeadingDegrees() {
    return getHeading().getDegrees();
  }

  /**
   * Returns current heading.
   *
   * @return heading as Rotation2d
   */
  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
  }

  /**
   * Optimizes and then sets heading, intended for Shuffleboard use.
   *
   * @param desiredRotationDegrees desired heading in degrees
   */
  @Config(name = "set rotation")
  public void setHeadingDegrees(double desiredRotationDegrees) {
    Rotation2d currentRotation = this.getHeading();
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
    Rotation2d currentRotation = this.getHeading();
    double optimizedRotationRadians = optimizedDesiredRotation.getRadians();
    double currentRotationRadians = currentRotation.getRadians();

    double deltaRadians = optimizedRotationRadians - currentRotationRadians;
    double deltaPulses = deltaRadians / kTurningRadiansPerPulse;

    double currentPulses = m_turningMotor.getSelectedSensorPosition();
    double referencePulses = currentPulses + deltaPulses;

    m_turningMotor.set(ControlMode.MotionMagic, referencePulses);
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
  @Config(name = "Set speed meters per second")
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
