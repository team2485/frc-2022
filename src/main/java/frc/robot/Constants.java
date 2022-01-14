// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

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
        public static final double ksDriveVolts = 0.477;
        public static final double kvDriveVoltSecondsPerMeter = 2.98;
        public static final double kaDriveVoltSecondsSquaredPerMeter = 0.147;

        //Turning control constants
        public static final double kTurningCurrentLimitAmps = 20;

        ////Turning mechanism/encoder constants
        public static final double kTurningGearRatio = 12.8; //motor turns per shaft turns
        public static final double kTurningRadiansPerMotorRev = 2*Math.PI/kTurningGearRatio;
        public static final double kTurningRadiansPerPulse = kTurningRadiansPerMotorRev/kFalconCPR;

        ////Turning feedforward constants
        public static final double ksTurningVolts = 0.60572;
        public static final double kvTurningVoltSecondsPerMeter = 0.20717;
        public static final double kaTurningVoltSecondsSquaredPerMeter = 0.0068542;

        ////Turning PID constants
        public static final double kPTurning = 0.2;
        public static final double kDTurning = 0.1;
        public static final double kFTurning = 0.4*1023/8360;

        ////Turning trapezoidal motion profile/motion magic constants
        public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16*Math.PI;
        public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256*Math.PI;
        public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond/kTurningRadiansPerPulse * 0.1;
        public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared/kTurningRadiansPerPulse * 0.01;

    }

    public static final class DriveConstants {
        /**
         * Zeros found with bevel gears facing right. 
         * Applied offset is the negative of the zero. 
         */
        public static final int kPigeonPort = 9;

        public static final int kFLDriveTalonPort = 1;
        public static final int kFLTurningTalonPort = 2;
        public static final int kFLCANCoderPort = 11;
        public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(-3.6);

        public static final int kFRDriveTalonPort = 3;
        public static final int kFRTurningTalonPort = 4;
        public static final int kFRCANCoderPort = 12;
        public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-167.7);

        public static final int kBRDriveTalonPort = 5;
        public static final int kBRTurningTalonPort = 6;
        public static final int kBRCANCoderPort = 13;
        public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(56.0);

        public static final int kBLDriveTalonPort = 7;
        public static final int kBLTurningTalonPort = 8;
        public static final int kBLCANCoderPort = 14;
        public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(139.0);



    }
}
