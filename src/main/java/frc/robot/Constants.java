// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.team2485.WarlordsLib.IDManager;
import frc.team2485.WarlordsLib.sendableRichness.SR_TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Include units in Constant names whenever possible/convenient. If not, include a comment
 * designating units.
 */
public final class Constants {
  public static final String kRobotIdFile = "/home/lvuser/id.txt";
  public static final String kCurrentLogFolder = "/home/lvuser/currentLogs";
  public static final double kNominalVoltage = 12.0;
  public static final int kFalconCPR = 2048; // pulses per rotation
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.15;
    public static final double kDriverLeftXDeadband = 0.08;
    public static final double kDriverLeftYDeadband = 0.08;

    public static final double kTriggerThreshold = 0.1;
  }

  public static final class AutoConstants {
    public static final double kAutoMaxSpeedMetersPerSecond = 3;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared =
        (kNominalVoltage
                - ModuleConstants.ksDriveVolts
                - kAutoMaxSpeedMetersPerSecond * ModuleConstants.kvDriveVoltSecondsPerMeter)
            / ModuleConstants.kaDriveVoltSecondsSquaredPerMeter;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        kAutoMaxSpeedMetersPerSecond / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared =
        kAutoMaxAccelerationMetersPerSecondSquared / DriveConstants.kTurningRadiusMeters;

    public static final double kPAutoXController = 1;
    public static final double kPAutoYController = 1;
    public static final double kPAutoThetaController = 5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kAutoMaxAngularSpeedRadiansPerSecond,
            kAutoMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ModuleConstants {
    // Drive control constants
    public static final double kDriveCurrentLimitAmps = 35;

    //// Drive mechanism/encoder constants
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = 0.1016 * Math.PI;
    public static final double kDriveGearRatio =
        IDManager.getInstance().select(6.75, 8.16); // motor turns per wheel turns
    public static final double kDriveDistMetersPerMotorRev =
        kWheelCircumferenceMeters / kDriveGearRatio;
    public static final double kDriveDistMetersPerPulse = kDriveDistMetersPerMotorRev / kFalconCPR;
    //// NOTE: CTRE Encoders return velocity in units/100 ms. CTRE velocity readings should be
    // multiplied by 10 to be per second.
    //// Drive feedforward constants
    // Field Carpet characterization constants
    // public static final double ksDriveVolts = 0.66707;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7887;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.29537;

    // Practice carpet characterization constants
    // public static final double ksDriveVolts = 0.667;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7695;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.23776;

    // practice carpet
    public static final double ksDriveVolts = IDManager.getInstance().select(0.0, 0.54);
    public static final double kvDriveVoltSecondsPerMeter =
        IDManager.getInstance().select(0.0, 2.5856);
    public static final double kaDriveVoltSecondsSquaredPerMeter =
        IDManager.getInstance().select(0.0, 0.31789);

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.05;
    // Turning control constants
    public static final double kTurningCurrentLimitAmps = 5;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse = kTurningRadiansPerMotorRev / kFalconCPR;

    //// Turning feedforward constants (unused in current implementation but useful for max speed)
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerRadian = 0.20717;
    public static final double kaTurningVoltSecondsSquaredPerRadian = 0.0068542;

    //// Turning PID constants
    public static final double kPTurning = 0.07;
    public static final double kDTurning = 0.01;
    public static final double kFTurning = 0.4 * 1023 / 8360;

    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared =
        (kNominalVoltage
                - ksTurningVolts
                - kModuleMaxSpeedTurningRadiansPerSecond * kvTurningVoltSecondsPerRadian)
            / kaTurningVoltSecondsSquaredPerRadian;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms =
        kModuleMaxSpeedTurningRadiansPerSecond / kTurningRadiansPerPulse * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared =
        kModuleMaxAccelerationTurningRadiansPerSecondSquared / kTurningRadiansPerPulse * 0.01;
  }

  public static final class DriveConstants {
    // Ports and zeros
    /** Zeros found with bevel gears facing right. Applied offset is the negative of the zero. */
    public static final int kPigeonPort = 9;

    public static final int kFLDriveTalonPort = 1;
    public static final int kFLTurningTalonPort = 2;
    public static final int kFLCANCoderPort = 10;
    public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(-3.6 - 5.18);

    public static final int kFRDriveTalonPort = 3;
    public static final int kFRTurningTalonPort = 4;
    public static final int kFRCANCoderPort = 11;
    public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-167.7 + 4.33);

    public static final int kBRDriveTalonPort = 5;
    public static final int kBRTurningTalonPort = 6;
    public static final int kBRCANCoderPort = 12;
    public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(56.0 - 1.66);

    public static final int kBLDriveTalonPort = 7;
    public static final int kBLTurningTalonPort = 8;
    public static final int kBLCANCoderPort = 13;
    public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(139.0 + 0.52);

    // Drivebase dimensions
    public static final double kWheelbaseLengthMeters = 0.635; // meters
    public static final double kWheelbaseWidthMeters = 0.508; // meters

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelbaseLengthMeters / 2, kWheelbaseWidthMeters / 2),
            new Translation2d(kWheelbaseLengthMeters / 2, -kWheelbaseWidthMeters / 2),
            new Translation2d(-kWheelbaseLengthMeters / 2, kWheelbaseWidthMeters / 2),
            new Translation2d(-kWheelbaseLengthMeters / 2, -kWheelbaseWidthMeters / 2));

    public static final double kTurningRadiusMeters =
        Math.sqrt(Math.pow(kWheelbaseLengthMeters / 2, 2) + Math.pow(kWheelbaseWidthMeters / 2, 2));

    // Max speed teleoperated
    public static final double kTeleopMaxSpeedMetersPerSecond = 3; // meters per second
    public static final double kTeleopMaxAngularSpeedRadiansPerSecond =
        kTeleopMaxSpeedMetersPerSecond / kTurningRadiusMeters; // radians per second

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared =
        kTeleopMaxAccelerationMetersPerSecondSquared / kTurningRadiusMeters;

    // Vision pose estimation constants
    public static final double kVisionWeightPerSec =
        0.85; // After one second, what % of pose average should be vision (4% in weighted avg)

    public static final double kVisionMaxAngularVelocityRadians =
        Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected

    public static final int kPoseHistoryCapacity = 500;

    public static final double kPRotation = 2;
    public static final double kRotationTolerance = 0.05;
  }

  public static final class FieldConstants {
    // ALL CONSTANTS IN METERS

    public static final double kVisionTargetHeightLower =
        Units.inchesToMeters(8 * 12 + 5.625); // Bottom of tape

    public static final double kVisionTargetHeightUpper =
        kVisionTargetHeightLower + Units.inchesToMeters(2);

    public static final double kVisionTargetRadius = Units.inchesToMeters(4.0 * 12.0 + 5.375) / 2;
    // All coordinates start at blue terminal corner (0,0)
    // Driver station to driver station is x

    public static final Translation2d kHubCenterTranslation = new Translation2d(8.2296, 4.1148);
    public static final Pose2d kHubCenterPosition =
        new Pose2d(kHubCenterTranslation, new Rotation2d(0));
  }

  public static final class VisionConstants {
    public static final String kCameraName = "gloworm";
    public static final double kPAngle = 0.05;
    public static final double kDAngle = 0.01;

    public static final double kVisionNominalFramerate = 45;

    public static final double kLensHeightMeters = 0.11;
    public static final double kLensPitchRadians = Math.toRadians(16.9);
    public static final Rotation2d kCameraPitch = new Rotation2d(kLensPitchRadians);

    // width of camera FOV (angle)
    public static final Rotation2d kCameraFOVW = Rotation2d.fromDegrees(59.6);

    // height of camera FOV (angle)
    public static final Rotation2d kCameraFOVH = Rotation2d.fromDegrees(49.7);

    // How much do the height and width of the camera vieport change for every meter out from the
    // camera?
    public static final double kCameraViewportRatioW = 2 * kCameraFOVW.times(0.5).getTan();
    public static final double kCameraViewportRatioH = 2 * kCameraFOVH.times(0.5).getTan();

    public static final int kCameraPixelsX = 960;
    public static final int kCameraPixelsY = 720;

    public static final int kMinTargetCount = 2;
    public static final double kCircleFitPrecision = 0.01;

    public static final double kExtraLatencySecs = 0.06;

    // idle behavior
    public static final double kTargetGraceSecs =
        0.5; // how long after target loss to wait for reaquire before turning to blink mode
    public static final double kBlinkPeriodSecs = 3.0;
    public static final double kBlinkLengthSecs = 0.5;

    public static final Transform2d kRobotToTurretCenterMeters =
        new Transform2d(
            new Translation2d(0.4, 0), // in meters
            new Rotation2d());

    public static final Translation2d kTurretCentertoCameraMeters = new Translation2d(0, 0);
  }

  public static final class TurretConstants {
    public static final double kTurretLoopTimeSeconds = 0.020;
    public static final int kTurretTalonPort = 32;
    public static final int kTurretSupplyCurrentLimitAmps = 5;
    public static final int kTurretSupplyCurrentThresholdAmps = 10;
    public static final int kTurretSupplyCurrentThresholdTimeMs = 1;

    public static final int kTurretEncoderChannel = 0; // Analog channel
    public static final double kTurretRangeOfMotion = 2 * Math.PI;
    public static final double kTurretOffset = 0;

    public static final double kVTurretVoltSecondsPerMeter = 0;
    public static final double kATurretVoltSecondsSquaredPerMeter = 0;
    public static final double kSTurretVolts = 0;

    public static final double kTurretVelocityRMSE = 0;
    public static final double kTurretAcclerationRMSE = 0;

    public static final double kTurretPotentiometerStdDev =
        0; // find voltage noise under ordinary operation

    // Q elements
    public static final double kTurretPositionToleranceRadians = Units.degreesToRadians(1.0);
    public static final double kTurretVelocityToleranceRadiansPerSecond =
        Units.degreesToRadians(10.0);

    // R elements
    public static final double kTurretVoltageTolerance = 12.0;

    public static final double kTurretMaxVelocityRadiansPerSecond = 0;
    public static final double kTurretMaxAccelerationRadiansPerSecondSquared =
        new SimpleMotorFeedforward(
                kSTurretVolts, kVTurretVoltSecondsPerMeter, kATurretVoltSecondsSquaredPerMeter)
            .maxAchievableAcceleration(kNominalVoltage, kTurretMaxVelocityRadiansPerSecond);

    public static final SR_TrapezoidProfile.Constraints kTurretMotionProfileConstraints =
        new SR_TrapezoidProfile.Constraints(
            kTurretMaxVelocityRadiansPerSecond, kTurretMaxAccelerationRadiansPerSecondSquared);

    public static final double kMinPositionRadians = 0;
    public static final double kMaxPositionRadians = 0;

    public static final double kBufferSizeRadians = Math.toRadians(25);
  }

  public static final class ShooterConstants {
    public static final int kShooterTalonPort = 30;

    public static final double kShooterStatorCurrentLimitAmps = 50;
    public static final double kShooterStatorCurrentThresholdAmps = 55;
    public static final double kShooterStatorCurrentThresholdTimeSecs = 0.05;

    public static final int kRevEncoderPulsesPerRevolution = 2048;
    public static final int kRevEncoderSamplesToAverage = 5;
    public static final int kFalconPulsesPerRevolution = 2048;

    public static final double kShooterRotationsPerPulse = 1.0 / kFalconPulsesPerRevolution;

    // shooter wood prototype gains
    public static final double kSShooterVolts = 0.65884;
    public static final double kVShooterVoltSecondsPerMeter = 0.11065;
    public static final double kAShooterVoltSecondsSquaredPerMeter = 0.023167;

    public static final double kShooterFeedforwardScale = 0.89;

    // currently unused
    public static final double kP = 1;
    public static final double kD = 0;

    public static final double kVelocityTolerance = 1.5;

    public static final double kShooterVelocityDipThresholdRotationsPerSecond = 5;
    public static final int kShooterVelocityDipRollingAverageWindow = 5;
  }

  public static final class IndexerConstants {
    public static final int kLowIndexerSparkPort = 22;
    public static final int kHighIndexerSparkPort = 23;

    public static final int kIndexerSmartCurrentLimitAmps = 20;
    public static final int kIndexerImmediateCurrentLimitAmps = 25;

    public static final double kLowIndexerPercentOutputIn = 0.5;

    public static final double kHighIndexerPercentOutputFeedToShooter = 0.7;
    public static final double kLowIndexerPercentOutputFeedToShooter = 0.5;
  }

  public static final class IntakeConstants {
    public static final int kIntakeSparkPort = 15;
    public static final int kIntakeSmartCurrentLimitAmps = 5;
    public static final int kIntakeImmediateCurrentLimitAmps = 10;

    public static final double kIntakePercentOutputIn = 0.5;
    public static final double kIntakePercentOutputOut = -0.5;

    public static final I2C.Port kI2CPort = I2C.Port.kOnboard;

    public static final Color kBlueBallColor = new Color(0.26, 0.42, 0.32);
    public static final Color kRedBallColor = new Color(0.35, 0.40, 0.25);

    public static final double kColorConfidenceLevel = 0.97;
  }

  public static final class IntakeArmConstants {
    public static final int kIntakeArmSparkPort = 0;
    public static final int kIntakeArmSmartCurrentLimitAmps = 5;
    public static final int kIntakeArmImmediateCurrentLimitAmps = 10;
    public static final double kIntakeArmPercentOutputUp = 0.5;
    public static final double kIntakeArmPercentOutputDown = -0.5;
  }

  public static final class HoodConstants {
    public static final int kHoodSparkPort = 31;
    public static final double kHoodGearRatio = 225; // motor turns : output/full hood turns
    public static final double kHoodRadiansPerMotorRev = 2 * Math.PI / kHoodGearRatio;

    public static final double kHoodBottomPositionRadians = 0.4363323; // from horizontal
    public static final double kHoodTopPositionRadians = 0.65;

    public static final int kHoodSmartCurrentLimitAmps = 2;
    public static final int kHoodImmediateCurrentLimitAmps = 5;

    // Hood characterization constants
    public static final double kSHoodVolts = 0.13428;
    public static final double kGHoodVolts = 0.061339;
    public static final double kVHoodVoltSecondsPerRadian = 0.99111;
    public static final double kAHoodVoltSecondsSquaredPerRadian = 0.12369;

    public static final double kHoodMaxSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kHoodMaxAccelerationRadiansPerSecondSquared =
        new ArmFeedforward(
                kSHoodVolts,
                kGHoodVolts,
                kVHoodVoltSecondsPerRadian,
                kAHoodVoltSecondsSquaredPerRadian)
            .maxAchievableAcceleration(
                kNominalVoltage, kHoodBottomPositionRadians, kHoodBottomPositionRadians);

    public static final SR_TrapezoidProfile.Constraints kHoodMotionProfileConstraints =
        new SR_TrapezoidProfile.Constraints(
            kHoodMaxSpeedRadiansPerSecond, kHoodMaxAccelerationRadiansPerSecondSquared);
    // Hood PID constants
    public static final double kPHood = 50;
    public static final double kDHood = 0;
    public static final double kHoodControllerPositionTolerance = 0.005;

    public static final double kHoodZeroingPercentOutput = -0.3;
  }
}
