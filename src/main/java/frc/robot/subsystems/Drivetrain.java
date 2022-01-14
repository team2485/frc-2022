package frc.robot.subsystems;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Drivetrain extends SubsystemBase implements Loggable{
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final PigeonIMU m_pigeon; 

    public Drivetrain() {  
        m_frontLeftModule = new SwerveModule(kFLDriveTalonPort, kFLTurningTalonPort, kFLCANCoderPort, kFLCANCoderZero, "FL");
        m_frontRightModule = new SwerveModule(kFRDriveTalonPort, kFRTurningTalonPort, kFRCANCoderPort, kFRCANCoderZero, "FR");
        m_backLeftModule = new SwerveModule(kBLDriveTalonPort, kBLTurningTalonPort, kBLCANCoderPort, kBLCANCoderZero, "BL");
        m_backRightModule = new SwerveModule(kBRDriveTalonPort, kBRTurningTalonPort, kBRCANCoderPort, kBRCANCoderZero, "BR");
        
        m_pigeon = new PigeonIMU(kPigeonPort);

    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] states =
        kDriveKinematics.toSwerveModuleStates(
          fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);

      //  m_frontLeftModule.setDesiredState(states[0]);
       // m_frontRightModule.setDesiredState(states[1]);
        m_backLeftModule.setDesiredState(states[2]);
       // m_backRightModule.setDesiredState(states[3]);

    }

    @Override
    public void periodic() {
        //this.drive(0, 0, 2, false);
    }

    
}
