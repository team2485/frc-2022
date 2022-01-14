package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;

public class Drivetrain extends SubsystemBase implements Loggable{
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final PigeonIMU m_pigeon; 

    private final SwerveDriveOdometry m_odometry;

    @Log
    private double desiredRotation;
    @Log
    private double desiredXSpeed;
    @Log
    private double desiredYSpeed;

    public Drivetrain() {  
        m_frontLeftModule = new SwerveModule(kFLDriveTalonPort, kFLTurningTalonPort, kFLCANCoderPort, kFLCANCoderZero, "FL");
        m_frontRightModule = new SwerveModule(kFRDriveTalonPort, kFRTurningTalonPort, kFRCANCoderPort, kFRCANCoderZero, "FR");
        m_backLeftModule = new SwerveModule(kBLDriveTalonPort, kBLTurningTalonPort, kBLCANCoderPort, kBLCANCoderZero, "BL");
        m_backRightModule = new SwerveModule(kBRDriveTalonPort, kBRTurningTalonPort, kBRCANCoderPort, kBRCANCoderZero, "BR");
        
        m_pigeon = new PigeonIMU(kPigeonPort);

        m_odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] states =
        kDriveKinematics.toSwerveModuleStates(
          fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);

        m_frontLeftModule.setDesiredState(states[0]);
        m_frontRightModule.setDesiredState(states[1]);
        m_backLeftModule.setDesiredState(states[2]);
        m_backRightModule.setDesiredState(states[3]);

        this.desiredRotation = rot;
        this.desiredXSpeed = xSpeed;
        this.desiredYSpeed = ySpeed;

    }

    //used in autonomous
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kAutoMaxSpeedMetersPerSecond);
        m_frontLeftModule.setDesiredState(desiredStates[0]);
        m_frontRightModule.setDesiredState(desiredStates[1]);
        m_backLeftModule.setDesiredState(desiredStates[2]);
        m_backRightModule.setDesiredState(desiredStates[3]);
    }
    
    public Pose2d getPoseMeters(){
        return m_odometry.getPoseMeters();
    }
        
    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
    }

    @Log
    public double getHeading() {
        return m_pigeon.getFusedHeading();
    }

    // Zeroes the heading of the robot
    public void zeroHeading() {
        m_pigeon.setFusedHeading(0);
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_pigeon.getFusedHeading()),
            m_frontLeftModule.getState(),
            m_backRightModule.getState(),
            m_frontRightModule.getState(),
            m_backRightModule.getState());
    }

    
}
