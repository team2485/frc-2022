package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonFX;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule implements Loggable {

    private final WPI_TalonFX m_driveMotor;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ksDriveVolts, kvDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter);

    private final WPI_TalonFX m_turningMotor;
    private final SimpleMotorFeedforward m_turningFeedforward = new SimpleMotorFeedforward(ksTurningVolts, kvTurningVoltSecondsPerMeter, kaTurningVoltSecondsSquaredPerMeter);
    private final CANCoder m_turningEncoder;
    
    private final String m_moduleID;

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, Rotation2d zero, String moduleID) {
        this.m_moduleID = moduleID;

        //Drive motor configuration
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
        driveMotorConfig.supplyCurrLimit.currentLimit = kDriveCurrentLimitAmps;
        driveMotorConfig.supplyCurrLimit.enable = true;
        driveMotorConfig.slot0.kP = kPDrive;
        this.m_driveMotor = new WPI_TalonFX(driveMotorID);
        m_driveMotor.configAllSettings(driveMotorConfig);
        m_driveMotor.enableVoltageCompensation(true);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        
        //Turning motor configuration
        TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();
        turningMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
        turningMotorConfig.supplyCurrLimit.currentLimit = kTurningCurrentLimitAmps;
        turningMotorConfig.supplyCurrLimit.enable = true;
        turningMotorConfig.slot0.kP = kPTurning;
        turningMotorConfig.slot0.kD = kDTurning;
        turningMotorConfig.slot0.kF = kFTurning;
        turningMotorConfig.motionCruiseVelocity = kModuleMaxSpeedTurningPulsesPer100Ms;
        turningMotorConfig.motionAcceleration = kModuleMaxAccelerationTurningPulsesPer100MsSquared;
        this.m_turningMotor = new WPI_TalonFX(turningMotorID);        
        m_turningMotor.configAllSettings(turningMotorConfig);
        m_turningMotor.enableVoltageCompensation(true);
        m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        m_turningMotor.setNeutralMode(NeutralMode.Brake);
        
        CANCoderConfiguration turningEncoderConfig = new CANCoderConfiguration();
        turningEncoderConfig.magnetOffsetDegrees = -zero.getDegrees();
        turningEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        m_turningEncoder = new CANCoder(turningEncoderID);
        m_turningEncoder.configAllSettings(turningEncoderConfig);
    
    }

    public void setDriveNeutralMode(NeutralMode mode) {
        m_driveMotor.setNeutralMode(mode);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getTurningPosition());
        this.setSpeedMetersPerSecond(state.speedMetersPerSecond);
        this.setTurningPosition(state.angle);

    }

    @Log(name = "current rotation")
    public double getTurningPositionDegrees() {
        return getTurningPosition().getDegrees();
    }
    private Rotation2d getTurningPosition() {
        return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition());
    }
    
    //-180 to 180
    @Config(name = "set rotation")
    public void setTurningPositionDegrees(double desiredRotationDegrees) {
        Rotation2d currentRotation = this.getTurningPosition();
        //use WPILib's swervemodulestate optimization to minimize change in heading
        Rotation2d optimizedDesiredRotation = SwerveModuleState.optimize(new SwerveModuleState(0, Rotation2d.fromDegrees(desiredRotationDegrees)), currentRotation).angle;
        this.setTurningPosition(optimizedDesiredRotation);
    }

    public void setTurningPosition(Rotation2d optimizedDesiredRotation) {
        Rotation2d currentRotation = this.getTurningPosition();
        double optimizedRotationRadians = optimizedDesiredRotation.getRadians();
        double currentRotationRadians = currentRotation.getRadians();

        double deltaRadians = optimizedRotationRadians - currentRotationRadians;
        double deltaPulses = deltaRadians / kTurningRadiansPerPulse;

        double currentPulses = m_turningMotor.getSelectedSensorPosition();
        double referencePulses = currentPulses + deltaPulses;

        m_turningMotor.set(ControlMode.MotionMagic, referencePulses);
    }

    


    @Log(name = "Speed meters per second")
    private double getSpeedMetersPerSecond() {
        return m_driveMotor.getSelectedSensorVelocity() * kDriveDistMetersPerPulse * 10;
    }

    @Config(name = "Set speed meters per second")
    public void setSpeedMetersPerSecond(double desiredSpeed) {
        SmartDashboard.putNumber("feedforward output",  m_driveFeedforward.calculate(
            this.getSpeedMetersPerSecond(), desiredSpeed
        ));

        SmartDashboard.putNumber("desired speed", desiredSpeed);

        m_driveMotor.set(ControlMode.Velocity, desiredSpeed/kDriveDistMetersPerPulse * 0.1, DemandType.ArbitraryFeedForward, m_driveFeedforward.calculate(desiredSpeed) / Constants.kNominalVoltage);
        //m_driveMotor.set(ControlMode.PercentOutput, m_driveFeedforward.calculate(desiredSpeed) / Constants.kNominalVoltage);
    }

    public void resetDriveEncoder() {
        m_driveMotor.setSelectedSensorPosition(0);
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
        int[] size = {3,4};
        return size;
      }


}
