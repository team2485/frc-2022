package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.CtreUtils;
import frc.team2485.WarlordsLib.motorcontrol.PIDTalonFX;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule implements Loggable {

    private final WPI_TalonFX m_driveMotor;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ksVoltsDrive, kvVoltSecondsPerMeterDrive, kaVoltSecondsSquaredPerMeterDrive);

    //private final WPI_TalonFX m_turningMotor;
    //private final SimpleMotorFeedforward m_turningFeedforward = new SimpleMotorFeedforward(ksVoltsTurning, kvVoltSecondsPerMeterTurning)
    
    private final String m_moduleID;

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, Rotation2d zero, String moduleID) {
        this.m_moduleID = moduleID;

        //Drive motor configuration
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.voltageCompSaturation = Constants.kNominalVoltage;
        driveMotorConfig.supplyCurrLimit.currentLimit = kDriveCurrentLimitAmps;
        driveMotorConfig.supplyCurrLimit.enable = true;
        this.m_driveMotor = new WPI_TalonFX(driveMotorID);
        CtreUtils.checkCtreError(m_driveMotor.configAllSettings(driveMotorConfig), "Failed to configure drive Falcon 500");
        m_driveMotor.enableVoltageCompensation(true);
        
    
    }

    public void setDriveNeutralMode(NeutralMode mode) {
        m_driveMotor.setNeutralMode(mode);
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

        m_driveMotor.set(ControlMode.PercentOutput, m_driveFeedforward.calculate(
            this.getSpeedMetersPerSecond(), desiredSpeed
        ) / Constants.kNominalVoltage);
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


}
