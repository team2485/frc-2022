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
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final int kFalconCPR = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final int kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriveSlewRate =
        3; // units per second to limit rate to, inverse of how long it will take from 0 to 1

    public static final double kDriverRightXDeadband = 0.15;
    public static final double kDriverLeftXDeadband = 0.08;
    public static final double kDriverLeftYDeadband = 0.05;

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
    public static final double kDriveCurrentLimitAmps = 60;

    //// Drive mechanism/encoder constants
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveGearRatio = 8.16; // motor turns per wheel turns
    public static final double kDriveDistMetersPerMotorRev =
        (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
    public static final double kDriveDistMetersPerPulse = kDriveDistMetersPerMotorRev / kFalconCPR;
    //// NOTE: CTRE Encoders return velocity in units/100 ms. CTRE velocity readings should be
    // multiplied by 10 to be per second.
    //// Drive feedforward constants
    // Field Carpet characterization constants
    // public static final double ksDriveVolts = 0.66707;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7887;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.29537;

    // Practice carpet characterization constants
    public static final double ksDriveVolts = 0.3;
    public static final double kvDriveVoltSecondsPerMeter = 2.7695;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.23776;

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.05;
    // Turning control constants
    public static final double kTurningCurrentLimitAmps = 60;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse = kTurningRadiansPerMotorRev / kFalconCPR;

    //// Turning feedforward constants (unused in current implementation but useful for max speed)
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerMeter = 0.20717;
    public static final double kaTurningVoltSecondsSquaredPerMeter = 0.0068542;

    //// Turning PID constants
    public static final double kPTurning = 0.1;
    public static final double kDTurning = 0.02;
    public static final double kFTurning = 0.4 * 1023 / 8360;

    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared =
        (kNominalVoltage
                - ksTurningVolts
                - kModuleMaxSpeedTurningRadiansPerSecond * kvTurningVoltSecondsPerMeter)
            / kaTurningVoltSecondsSquaredPerMeter;
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
    public static final double kTeleopMaxSpeedMetersPerSecond = 2; // meters per second
    public static final double kTeleopMaxAngularSpeedRadiansPerSecond =
        kTeleopMaxSpeedMetersPerSecond / kTurningRadiusMeters; // radians per second

    // Vision pose estimation constants
    public static final double kVisionWeightPerSec =
        0.85; // After one second, what % of pose average should be vision (4% in weighted avg)

    public static final double kVisionMaxAngularVelocityRadians =
        Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected

    public static final int kPoseHistoryCapacity = 500;

    public static final double kPRotationHubTracking = 2;
    public static final double kRotationToleranceHubTracking = 0.05;
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

    public static final Transform2d kRobotToCameraMeters =
        new Transform2d(
            new Translation2d(0.4, 0), // in meters
            new Rotation2d());
  }

  public static final class FlywheelConstants {
    public static final int kFlywheelTalonPort = 11;

    public static final double kFlywheelTalonCurrentLimit = 50;

    public static final int kRevEncoderPulsesPerRevolution = 2048;
    public static final int kRevEncoderSamplesToAverage = 5;
    public static final int kFalconPulsesPerRevolution = 2048;

    public static final double kFlywheelRotationsPerPulse = 1.0 / kFalconPulsesPerRevolution;

    public static final double kFlywheelMaxSpeedRotationsPerSecond = 30;

    // shooter wood prototype gains
    public static final double kSVolts = 0.65884;
    public static final double kVVoltSecondsPerMeter = 0.11065;
    public static final double kAVoltSecondsPerMeterSquared = 0.023167;

    // currently unused
    public static final double kP = 1;
    public static final double kD = 0;

    public static final double kVelocityTolerance = 0.5;
  }

  public static final class ClimbElevatorConstants {
    public static final double kElevatorControlLoopTimeSeconds = 0.01;
    // SLIDE CONSTANTS
    public static final int kElevatorTalonPort = 40;
    public static final double kElevatorSupplyCurrentLimitAmps = 25;
    public static final double kElevatorSupplyCurrentThresholdAmps = 30;
    public static final double kElevatorSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kElevatorStatorCurrentLimitAmps = 20;
    public static final double kElevatorStatorCurrentThresholdAmps = 25;
    public static final double kElevatorStatorCurrentThresholdTimeSecs = 0.05;

    public static final int kElevatorSlotSensorTopPort = 0; // dio
    public static final int kElevatorSlotSensorBottomPort = 1; // dio

    public static final double kElevatorTopStopPosition = Units.inchesToMeters(21.25);
    public static final double kElevatorBottomStopPosition = Units.inchesToMeters(0);

    public static final double kElevatorSlotSensorTopPosition = Units.inchesToMeters(20.75);
    public static final double kElevatorSlotSensorBottomPosition = Units.inchesToMeters(0.25);
    public static final double kSlotSensorDebounceTime = 0.1;

    public static final int kElevatorServoPort = 0; // pwm
    public static final double kElevatorServoRange = 270;
    public static final double kElevatorServoEngageValue = 0;
    public static final double kElevatorServoDisengageValue = 0.5;

    public static final double kSprocketCircumferenceMeters = 0.0323342 * Math.PI;

    public static final double kElevatorGearRatio = 32.67; // motor turns/pulley turns
    public static final double kElevatorDistancePerMotorRevMeters =
        kSprocketCircumferenceMeters / kElevatorGearRatio;

    public static final double kSlideDistancePerPulseMeters =
        kElevatorDistancePerMotorRevMeters / kFalconCPR;

    // Slide characterization constants: UNLOADED (not carrying robot)
    public static final double ksElevatorUnloadedVolts = 0.70015;
    public static final double kgElevatorUnloadedVolts = 0.010378;
    public static final double kvElevatorUnloadedVoltSecondsPerMeter = 30.626;
    public static final double kaElevatorUnloadedVoltSecondsSquaredPerMeter = 0.01;

    // Slide characterization constants: LOADED ( carrying robot)
    public static final double ksElevatorLoadedVolts = 0.44256;
    public static final double kgElevatorLoadedVolts =
        -0.50; // this is negative because gravity fights the downward motion when loaded --
    // retracting the elevator moves the robot up.
    public static final double kvElevatorLoadedVoltSecondsPerMeter = 30;
    public static final double kaElevatorLoadedVoltSecondsSquaredPerMeter =
        0.18; // these are recalc gains -- the ka from sysid was lost in the noise

    public static final double kPElevatorUnloadedVoltsPerMeter = 165;
    public static final double kDElevatorUnloadedVoltSecondsPerMeter = 20;

    public static final double kPElevatorLoadedVoltsPerMeter = 100;
    public static final double kDElevatorLoadedVoltSecondsPerMeter = 10;

    public static final double kElevatorPositionToleranceMeters = 0.005;
    public static final double kElevatorVelocityToleranceMetersPerSecond = 0.01;

    public static final double kSlideMaxSpeedMetersPerSecond = 0.5;
    // Find maximum simultaneously achievable acceleration
    public static final double kSlideMaxAccelerationMetersPerSecondSquaredUnloaded = 0.3;

    public static final double kSlideMaxAccelerationMetersPerSecondSquaredLoaded = 0.1;

    // Constraint for the motion profilied elevator controller (unloaded mode)
    public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraintsUnloaded =
        new SR_TrapezoidProfile.Constraints(
            kSlideMaxSpeedMetersPerSecond, kSlideMaxAccelerationMetersPerSecondSquaredUnloaded);

    // Constraint for the motion profilied elevator controller (loaded mode)
    public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraintsLoaded =
        new SR_TrapezoidProfile.Constraints(
            kSlideMaxSpeedMetersPerSecond, kSlideMaxAccelerationMetersPerSecondSquaredLoaded);
  }

  public static final class ClimbArmConstants {
    public static final double kArmControlLoopTimeSeconds = 0.01;

    // ARM CONSTANTS
    public static final int kArmTalonPort = 41;
    public static final double kArmSupplyCurrentLimitAmps = 25;
    public static final double kArmSupplyCurrentThresholdAmps = 30;
    public static final double kArmSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kArmStatorCurrentLimitAmps = 30;
    public static final double kArmStatorCurrentThresholdAmps = 35;
    public static final double kArmStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kArmStatorCurrentSpikeThresholdAmps = 30;
    public static final double kArmStatorCurrentSpikeDebounceTimeSeconds = 0.2;

    public static final double kArmGearRatio = 57.6; // motor turns/pinion turns
    public static final double kArmRotationsPerMotorRev = 1 / kArmGearRatio;
    public static final double kArmRotationsPerPulse = kArmRotationsPerMotorRev / kFalconCPR;

    public static final double kSprocketCircumferenceMeters = 0.0323342 * Math.PI;

    // arm characteristics used in angular mode
    public static final double kArmMomentOfIntertia = 5;
    public static final double kArmLengthMeters = 2; // length to center of mass
    public static final double kGravityMetersPerSecondSquared = 9.81;

    // arm characteristics used in translation mode
    public static final double kArmDistancePerMotorRevMeters =
        kSprocketCircumferenceMeters / kArmGearRatio;
    // public static final double kArmRotationToleranceRadians = 0.05;

    public static final double kArmRotationTolerance = 0.01;
    // ARM TRANSLATION CONSTANTS
    public static final double ksArmTranslationVolts = 0.55953;
    public static final double kgArmTranslationVolts = 0.18092;
    public static final double kvArmTranslationVoltSecondsPerMeter = 62.802;
    public static final double kaArmTranslationVoltSecondsSquaredPerMeter = 0.04;

    public static final double kArmMaxSpeedTranslationMetersPerSecond = 0.05;
    public static final double kArmMaxAccelerationTranslationMetersPerSecondSquared = 0.05;

    // Constraint for the motion profilied arm rotation controller
    public static final SR_TrapezoidProfile.Constraints kArmControllerConstraintsTranslation =
        new SR_TrapezoidProfile.Constraints(
            kArmMaxSpeedTranslationMetersPerSecond,
            kArmMaxAccelerationTranslationMetersPerSecondSquared);

    public static final double kPArmTranslationVoltsPerMeter = 400;
    public static final double kDArmTranslationVoltSecondsPerMeter = 80;

    public static final double ksArmUnloadedVolts = 0.6;
    public static final double kgArmUnloadedVolts = 0.02;
    public static final double kvArmUnloadedVoltSecondsPerMeter = 55.315;
    public static final double kaArmUnloadedVoltSecondsSquaredPerMeter = 0.01;

    public static final double kArmTranslationToleranceMeters = 0.02;
    public static final double kArmTranslationVelocityToleranceMetersPerSecond = 0.05;
  }
}
