// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *  
 * Include units in Constant names whenever possible/convenient.
 * If not, include a comment designating units. 
 */
public final class Constants {
    public static final String kRobotIdFile = "/home/lvuser/id.txt";
    public static final double kNominalVoltage = 12.0;
    public static final int kFalconCPR = 2048; //pulses per rotation
    public static final int kCANTimeoutMs = 250;


    public static final class OIConstants {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;

        public static final double kDriveSlewRate = 3; //units per second to limit rate to, inverse of how long it will take from 0 to 1

        public static final double kDriverRightXDeadband = 0.15;
        public static final double kDriverLeftXDeadband = 0.08;
        public static final double kDriverLeftYDeadband = 0.05;
    }

    public static final class AutoConstants{
        public static final double kAutoMaxSpeedMetersPerSecond = 3;
        public static final double kAutoMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kAutoMaxAngularSpeedRadiansPerSecond = 3*Math.PI;
        public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 3*Math.PI;
    
        public static final double kPAutoXController = 1;
        public static final double kPAutoYController = 1;
        public static final double kPAutoThetaController = 5;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kAutoMaxAngularSpeedRadiansPerSecond, kAutoMaxAngularAccelerationRadiansPerSecondSquared); 
    }

    public static final class ModuleConstants{
        //Drive control constants 
        public static final double kDriveCurrentLimitAmps = 80;

        ////Drive mechanism/encoder constants
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kDriveGearRatio = 8.16; //motor turns per wheel turns
        public static final double kDriveDistMetersPerMotorRev = (kWheelDiameterMeters*Math.PI)/kDriveGearRatio; 
        public static final double kDriveDistMetersPerPulse = kDriveDistMetersPerMotorRev/kFalconCPR;
        ////NOTE: CTRE Encoders return velocity in units/100 ms. CTRE velocity readings should be multiplied by 10 to be per second. 

        ////Drive feedforward constants
        public static final double ksDriveVolts = 0.66707;
        public static final double kvDriveVoltSecondsPerMeter = 2.7887;
        public static final double kaDriveVoltSecondsSquaredPerMeter = 0.29537;

        ////Drive PID constants
        public static final double kPDrive = 0.05;
        //Turning control constants
        public static final double kTurningCurrentLimitAmps = 20;

        ////Turning mechanism/encoder constants
        public static final double kTurningGearRatio = 12.8; //motor turns per shaft turns
        public static final double kTurningRadiansPerMotorRev = 2*Math.PI/kTurningGearRatio;
        public static final double kTurningRadiansPerPulse = kTurningRadiansPerMotorRev/kFalconCPR;

        ////Turning feedforward constants (unused in current implementation)
        public static final double ksTurningVolts = 0.60572;
        public static final double kvTurningVoltSecondsPerMeter = 0.20717;
        public static final double kaTurningVoltSecondsSquaredPerMeter = 0.0068542;

        ////Turning PID constants
        public static final double kPTurning = 0.2;
        public static final double kDTurning = 0.05;
        public static final double kFTurning = 0.4*1023/8360;

        ////Turning trapezoidal motion profile/motion magic constants
        public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16*Math.PI;
        public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256*Math.PI;
        public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond/kTurningRadiansPerPulse * 0.1;
        public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared/kTurningRadiansPerPulse * 0.01;

    }

    public static final class DriveConstants {
        //Ports and zeros
        /**
         * Zeros found with bevel gears facing right. 
         * Applied offset is the negative of the zero. 
         */
        public static final int kPigeonPort = 9;

        public static final int kFLDriveTalonPort = 1;
        public static final int kFLTurningTalonPort = 2;
        public static final int kFLCANCoderPort = 11;
        public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(-3.6-5.18);

        public static final int kFRDriveTalonPort = 3;
        public static final int kFRTurningTalonPort = 4;
        public static final int kFRCANCoderPort = 12;
        public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-167.7+4.33);

        public static final int kBRDriveTalonPort = 5;
        public static final int kBRTurningTalonPort = 6;
        public static final int kBRCANCoderPort = 13;
        public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(56.0-1.66);

        public static final int kBLDriveTalonPort = 7;
        public static final int kBLTurningTalonPort = 8;
        public static final int kBLCANCoderPort = 14;
        public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(139.0+0.52);

        //Max speed teleoperated
        public static final double kTeleopMaxSpeedMetersPerSecond = 2; //meters per second
        public static final double kTeleopMaxAngularSpeedRadiansPerSecond = Math.PI; //radians per second

        //Drivebase dimensions
        public static final double kWheelbaseLengthMeters = 0.635; //meters
        public static final double kWheelbaseWidthMeters = 0.508; //meters
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(
                kWheelbaseLengthMeters/2,
                kWheelbaseWidthMeters/2
            ), 
            new Translation2d(
                kWheelbaseLengthMeters/2,
                -kWheelbaseWidthMeters/2
            ), 
            new Translation2d(
                -kWheelbaseLengthMeters/2,
                kWheelbaseWidthMeters/2
            ), 
            new Translation2d(
                -kWheelbaseLengthMeters/2,
                -kWheelbaseWidthMeters/2
            )  
        );

     
    }

    public static final class FieldConstants {
        //ALL CONSTANTS IN METERS
        //THIS IS FOR A RECYCLING BIN -- CHANGE FOR REAL FIELD!
        public static final double kTargetHeightMeters = 0.58;

        //All coordinates start at blue terminal corner (0,0)
        //Driver station to driver station is x

        public static final Translation2d kHubCenterPositionMeters = new Translation2d(8.2296, 4.1148);
        public static final Pose2d kFieldToTargetMeters = new Pose2d(kHubCenterPositionMeters, new Rotation2d(0));

    }


    public static final class VisionConstants {
        public static final double kPAngle = 0.05;
        public static final double kDAngle = 0.01;

        public static final double kCameraHeightMeters = 0.11;
        public static final double kCameraPitchRadians = Math.toRadians(27);
        
        public static final Transform2d kCameraToRobotMeters = new Transform2d(
                                                                new Translation2d(-0.45, 0), // in meters
                                                                new Rotation2d());
        
    }
}
