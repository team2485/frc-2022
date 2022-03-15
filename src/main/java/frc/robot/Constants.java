// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.interpolation.ShotParameter;
import frc.team2485.WarlordsLib.IDManager;
import frc.team2485.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import java.util.Map;
import java.util.TreeMap;

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
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  // meters and shot parameters (radians and rps)
  public static final TreeMap<Double, ShotParameter> kShootingMap =
      new TreeMap<>(
          Map.ofEntries(
              entry(1.45, new ShotParameter(33, 0.47, 0)),
              entry(1.989, new ShotParameter(45, 0.45, 0)),
              entry(1.99, new ShotParameter(62, HoodConstants.kHoodBottomPositionRadians, 0)),
              entry(2.12, new ShotParameter(65, HoodConstants.kHoodBottomPositionRadians, 0)),
              entry(2.52, new ShotParameter(75, HoodConstants.kHoodBottomPositionRadians, 0)),
              entry(3.29, new ShotParameter(78, 0.46, 0)),
              entry(3.67, new ShotParameter(80, 0.46, 0)),
              entry(4.15, new ShotParameter(81.5, 0.47, 0)),
              entry(4.68, new ShotParameter(86.5, 0.48, 0)),
              entry(5.2, new ShotParameter(89, 0.5, 0)),
              entry(5.72, new ShotParameter(91, 0.52, 0))));

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final int kFalconCPR = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final int kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

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
        IDManager.getInstance().select(6.75, 6.86); // motor turns per wheel turns
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
    public static final double ksDriveVolts = IDManager.getInstance().select(0.67757, 0.73658);
    public static final double kvDriveVoltSecondsPerMeter =
        IDManager.getInstance().select(2.1986, 2.2729);
    public static final double kaDriveVoltSecondsSquaredPerMeter =
        IDManager.getInstance().select(0.30189, 0.37446);

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.1;
    // Turning control constants
    public static final double kTurningCurrentLimitAmps = 10;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse = kTurningRadiansPerMotorRev / kFalconCPR;

    //// Turning feedforward constants (unused in current implementation but useful for max speed)
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerRadian = 0.20717;
    public static final double kaTurningVoltSecondsSquaredPerRadian = 0.0068542;

    //// Turning PID constants
    public static final double kPTurning = 1;
    public static final double kDTurning = 0.1;
    public static final double kFTurning =
        IDManager.getInstance().select(0.4 * 1023 / 7674, 0.4 * 1023 / 8360);

    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 4 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 32 * Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms =
        kModuleMaxSpeedTurningRadiansPerSecond / kTurningRadiansPerPulse * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared =
        kModuleMaxAccelerationTurningRadiansPerSecondSquared / kTurningRadiansPerPulse * 0.01;
  }

  public static final class DriveConstants {
    // Ports and zeros
    /** Zeros found with bevel gears facing right. Applied offset is the negative of the zero. */
    public static final int kPigeonPort = 9;

    public static final int kFLDriveTalonPort = 5;
    public static final int kFLTurningTalonPort = 6;
    public static final int kFLCANCoderPort = 12;
    public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(56.0 - 1.66);

    public static final int kFRDriveTalonPort = 7;
    public static final int kFRTurningTalonPort = 8;
    public static final int kFRCANCoderPort = 13;
    public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(139.0 + 0.52);

    public static final int kBRDriveTalonPort = 1;
    public static final int kBRTurningTalonPort = 2;
    public static final int kBRCANCoderPort = 10;
    public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(-3.6 - 5.18);

    public static final int kBLDriveTalonPort = 3;
    public static final int kBLTurningTalonPort = 4;
    public static final int kBLCANCoderPort = 11;
    public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(-167.7 + 4.33);

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

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

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

    public static final double kRobotBumperLengthMeters = 0.97;
    public static final double kRobotBumperWidthMeters = 0.84;
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
        new Transform2d(new Translation2d(0.0508, 0), new Rotation2d()); // in meters

    public static final Translation2d kTurretCentertoCameraMeters = new Translation2d(0, 0);
  }

  public static final class IntakeConstants {
    public static final int kIntakeSparkPort = 21;
    public static final double kIntakeLoopTimeSeconds = 0.02;
    public static final int kIntakeSmartCurrentLimitAmps = 20;
    public static final int kIntakeImmediateCurrentLimitAmps = 25;

    public static final double kIntakeGearRatio = 4; // motor turns : output/full hood turns

    public static final double kIntakeFreeSpeedRotationsPerSecond =
        kNeo550FreeSpeedRotationsPerSecond / kIntakeGearRatio;

    public static final double kIntakeTopWheelDiameterMeters = 0.1016; // 4 in
    public static final double kIntakeBottomWheelDiameterMeters = 0.1524; // 6 in
    public static final double kIntakeBottomWheelCircumferenceMeters = 0.1524 * Math.PI;
    public static final double kIntakeDefaultSpeedRotationsPerSecond =
        kIntakeFreeSpeedRotationsPerSecond * 0.5;

    public static final double kSIntakeVolts = 0;
    public static final double kVIntakeVoltSecondsPerMeter = 0.1;
    public static final double kAIntakeVoltSecondsSquaredPerMeter = 0.01;

    public static final double kIntakeVelocityToleranceRotationsPerSecond = 1;

    public static final int kPhotoSensorPort = 5; // dio port

    public static final I2C.Port kI2CPort = I2C.Port.kOnboard;

    public static final Color kBlueBallColor = new Color(0.26, 0.42, 0.32);
    public static final Color kRedBallColor = new Color(0.35, 0.40, 0.25);

    public static final double kColorConfidenceLevel = 0.97;
  }

  public static final class IntakeArmConstants {
    public static final int kIntakeArmSparkPort = 20;
    public static final double kIntakeArmLoopTimeSeconds = 0.020;
    public static final int kIntakeArmSmartCurrentLimitAmps = 20;
    public static final int kIntakeArmImmediateCurrentLimitAmps = 25;

    public static final double kIntakeArmGearRatio = 125.0;
    public static final double kIntakeArmFreeSpeedRadiansPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kIntakeArmGearRatio * (2 * Math.PI);
    public static final double kIntakeArmRadiansPerMotorRev =
        1.0 / kIntakeArmGearRatio * 2 * Math.PI;

    public static final double kIntakeArmBottomPositionRadians = -0.2618; // from horizontal
    public static final double kIntakeArmTopPositionRadians = 2.0071; // change later

    // Intake Arm characterization constants
    public static final double kSIntakeArmVolts = 0.5;
    public static final double kGIntakeArmVolts = 1.34;
    public static final double kVIntakeArmVoltsSecondsPerRadian = 0.73;
    public static final double kAIntakeArmVoltsSecondsSquaredPerRadian = 0.06;

    public static final double kIntakeArmMaxSpeedRadiansPerSecond = 1;
    public static final double kIntakeArmMaxAccelerationRadiansPerSecondSquared = 0.5;
    // new ArmFeedforward(
    //         kSIntakeArmVolts,
    //         kGIntakeArmVolts,
    //         kVIntakeArmVoltsSecondsPerRadian,
    //         kAIntakeArmVoltsSecondsSquaredPerRadian)
    //     .maxAchievableAcceleration(
    //         kNominalVoltage,
    //         kIntakeArmBottomPositionRadians,
    //         kIntakeArmMaxSpeedRadiansPerSecond);

    public static final SR_TrapezoidProfile.Constraints kIntakeArmMotionProfileConstraints =
        new SR_TrapezoidProfile.Constraints(
            kIntakeArmMaxSpeedRadiansPerSecond, kIntakeArmMaxAccelerationRadiansPerSecondSquared);

    // Intake Arm PID constants
    public static final double kPIntakeArmVoltsPerRadian = 10;
    public static final double kDIntakeArmVoltSecondsPerRadian = 0;
    public static final double kIntakeArmPositionToleranceRadians = 0.01;
  }

  public static final class IndexerConstants {
    public static final int kIndexerSparkPort = 22;
    public static final double kIndexerLoopTimeSeconds = 0.02;
    public static final int kIndexerSmartCurrentLimitAmps = 20;
    public static final int kIndexerImmediateCurrentLimitAmps = 25;

    public static final double kIndexerGearRatio = 4; // motor turns : output/full hood turns

    public static final double kIndexerFreeSpeedRotationsPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kIndexerGearRatio;

    public static final double kIndexerEntryWheelDiameterMeters = 0.0508; // 2 inches

    public static final double kIndexerIntakeSpeedRatio =
        IntakeConstants.kIntakeTopWheelDiameterMeters / kIndexerEntryWheelDiameterMeters;

    public static final double kIndexerDefaultSpeedRotationsPerSecond =
        kIndexerFreeSpeedRotationsPerSecond * 0.7;

    public static final double kSIndexerVolts = 0.1;
    public static final double kVIndexerVoltSecondsPerMeter = 0.1;
    public static final double kAIndexerVoltSecondsSquaredPerMeter = 0.005;

    public static final double kIndexerVelocityToleranceRotationsPerSecond = 1;
  }

  public static final class FeederConstants {
    public static final int kFeederSparkPort = 23;
    public static final double kFeederLoopTimeSeconds = 0.02;
    public static final int kFeederSmartCurrentLimitAmps = 15;
    public static final int kFeederImmediateCurrentLimitAmps = 20;

    public static final double kFeederGearRatio = 4; // motor turns : output/full hood turns

    public static final double kFeederFreeSpeedRotationsPerSecond =
        kNeo550FreeSpeedRotationsPerSecond / kFeederGearRatio;

    public static final double kFeederDefaultSpeedRotationsPerSecond =
        kFeederFreeSpeedRotationsPerSecond / 2;

    public static final double kFeederPulleyCircumferenceMeters = 0.0191008 * Math.PI;

    public static final double kFeederSurfaceFreeSpeedMetersPerSecond =
        kFeederFreeSpeedRotationsPerSecond * kFeederPulleyCircumferenceMeters;

    public static final double kFeederShooterSurfaceSpeedRatio =
        kFeederSurfaceFreeSpeedMetersPerSecond
            / ShooterConstants.kShooterSurfaceFreeSpeedMetersPerSecond;

    public static final double kSFeederVolts = 0.65884;
    public static final double kVFeederVoltSecondsPerMeter = 0.11065;
    public static final double kAFeederVoltSecondsSquaredPerMeter = 0.023167;

    public static final double kFeederVelocityToleranceRotationsPerSecond = 1;

    public static final int kFeederServoPort = 1;
    public static final double kServoDisengagedPosition = 0.45;
    public static final double kServoEngagedPosition = 0.15;
  }

  public static final class HoodConstants {
    public static final int kHoodSparkPort = 31;
    public static final int kHoodSmartCurrentLimitAmps = 5;
    public static final int kHoodImmediateCurrentLimitAmps = 10;

    public static final double kHoodLoopTimeSeconds = 0.02;

    public static final double kHoodGearRatio = 225; // motor turns : output/full hood turns
    public static final double kHoodRadiansPerMotorRev = 2 * Math.PI / kHoodGearRatio;

    public static final double kHoodFreeSpeedRadiansPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kHoodGearRatio * (2 * Math.PI);

    public static final double kHoodBottomPositionRadians =
        0.4363323; // from horizontal (25.21 deg)
    public static final double kHoodTopPositionRadians = 0.525; //

    // Hood characterization constants
    public static final double ksHoodVolts = 0.13428;
    public static final double kgHoodVolts = 0.061339;
    public static final double kvHoodVoltSecondsPerRadian = 0.99111;
    public static final double kaHoodVoltSecondsSquaredPerRadian = 0.12369;

    public static final double kHoodMaxSpeedRadiansPerSecond = 2;
    public static final double kHoodMaxAccelerationRadiansPerSecondSquared = 10;

    public static final SR_TrapezoidProfile.Constraints kHoodMotionProfileConstraints =
        new SR_TrapezoidProfile.Constraints(
            kHoodMaxSpeedRadiansPerSecond, kHoodMaxAccelerationRadiansPerSecondSquared);
    // Hood PID constants
    public static final double kPHood = 50;
    public static final double kDHood = 0;
    public static final double kHoodPositionToleranceRadians = 0.005;

    public static final double kHoodSetpointDeadbandRadians = 0.001;

    public static final double kHoodZeroingVoltage = -0.5;
  }

  public static final class TurretConstants {
    public static final double kTurretLoopTimeSeconds = 0.02;
    public static final int kTurretTalonPort = 32;
    public static final int kTurretSupplyCurrentLimitAmps = 5;
    public static final int kTurretSupplyCurrentThresholdAmps = 10;
    public static final int kTurretSupplyCurrentThresholdTimeMs = 1;

    public static final int kTurretCCWSlotSensorPort = 2;
    public static final int kTurretCWSlotSensorPort = 3;

    public static final int kTurretPotentiometerChannel = 0; // Analog channel
    public static final double kTurretPotentiometerRangeOfMotion = Math.PI * 2;
    public static final double kTurretPotentiometerOffset =
        -Math.PI - 0.3625 - 0.48 + 0.053 - 0.16 - 1.37 + 1.196 - 0.225;

    public static final double kTurretGearing = 462;

    public static final double kTurretFreeSpeedRadiansPerSecond =
        k775FreeSpeedRotationsPerSecond / kTurretGearing * 2 * Math.PI;

    public static final double kVTurretVoltSecondsPerRadian = 1;
    public static final double kATurretVoltSecondsSquaredPerRadian = 0.1;
    public static final double kSTurretVolts = 0.9;

    public static final double kPTurretVoltsPerRadian = 8;
    public static final double kDTurretVoltSecondsPerRadian = 4;

    public static final double kTurretPositionTolerance = 0.02;

    public static final double kTurretMaxVelocityRadiansPerSecond = 4;
    public static final double kTurretMaxAccelerationRadiansPerSecondSquared = 2;

    public static final SR_TrapezoidProfile.Constraints kTurretMotionProfileConstraints =
        new SR_TrapezoidProfile.Constraints(
            kTurretMaxVelocityRadiansPerSecond, kTurretMaxAccelerationRadiansPerSecondSquared);

    public static final double kTurretMinPositionRadians = -0.4;
    public static final double kTurretMaxPositionRadians = 0.4;
    public static final double kTurretRangeRadians =
        kTurretMaxPositionRadians - kTurretMinPositionRadians;

    public static final double kBufferSizeRadians = Math.toRadians(25);
  }

  public static final class ShooterConstants {
    public static final int kShooterTalonPort = 30;

    public static final double kShooterLoopTimeSeconds = 0.01;
    public static final double kShooterTalonCurrentLimit = 50;

    public static final int kRevEncoderPulsesPerRevolution = 2048;
    public static final int kRevEncoderSamplesToAverage = 5;
    public static final int kFalconPulsesPerRevolution = 2048;

    public static final double kShooterRotationsPerPulse = 1.0 / kFalconPulsesPerRevolution;

    public static final double kShooterCircumferenceMeters = 0.1524 * Math.PI;
    public static final double kShooterFreeSpeedRotationsPerSecond = 93.2;
    public static final double kShooterSurfaceFreeSpeedMetersPerSecond =
        kShooterFreeSpeedRotationsPerSecond * kShooterCircumferenceMeters;

    // shooter wood prototype gains
    public static final double kSShooterVolts = 0.59077;
    public static final double kVShooterVoltSecondsPerMeter = 0.10773;
    public static final double kAShooterVoltSecondsSquaredPerMeter = 0.0091575;

    public static final double kShooterFeedforwardScale = 0.95;

    // currently unused
    public static final double kP = 1;
    public static final double kD = 0;

    public static final double kShooterControlVelocityTolerance = 0.5;

    public static final double kShooterFeedVelocityTolerance = 2.5;

    public static final double kShooterVelocityDipThresholdRotationsPerSecond = 3;
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

    public static final double kElevatorFreeSpeedMetersPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kElevatorGearRatio * kSprocketCircumferenceMeters;

    public static final double kSlideDistancePerPulseMeters =
        kElevatorDistancePerMotorRevMeters / kFalconCPR;

    // Slide characterization constants: UNLOADED (not carrying robot)
    public static final double ksElevatorUnloadedVolts = 0.70015;
    public static final double kgElevatorUnloadedVolts = 0.010378;
    public static final double kvElevatorUnloadedVoltSecondsPerMeter = 30;
    public static final double kaElevatorUnloadedVoltSecondsSquaredPerMeter = 0.01;

    // Slide characterization constants: LOADED ( carrying robot)
    public static final double ksElevatorLoadedVolts = 0.44256;
    public static final double kgElevatorLoadedVolts =
        -0.50; // this is negative because gravity fights the downward motion when loaded --
    // retracting the elevator moves the robot up.
    public static final double kvElevatorLoadedVoltSecondsPerMeter = 30;
    public static final double kaElevatorLoadedVoltSecondsSquaredPerMeter =
        0.18; // these are recalc gains -- the ka from sysid was lost in the noise

    public static final double kPElevatorUnloadedVoltsPerMeter = 100;
    public static final double kDElevatorUnloadedVoltSecondsPerMeter = 0.05;

    public static final double kPElevatorLoadedVoltsPerMeter = 100;
    public static final double kDElevatorLoadedVoltSecondsPerMeter = 10;

    public static final double kElevatorPositionToleranceMeters = 0.005;
    public static final double kElevatorVelocityToleranceMetersPerSecond = 0.01;

    public static final double kElevatorMaxSpeedMetersPerSecond =
        kElevatorFreeSpeedMetersPerSecond * 0.5;
    // Find maximum simultaneously achievable acceleration
    public static final double kElevatorMaxAccelerationMetersPerSecondSquaredUnloaded = 0.3;

    public static final double kSlideMaxAccelerationMetersPerSecondSquaredLoaded = 0.1;
    ;

    // Constraint for the motion profilied elevator controller (unloaded mode)
    public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraintsUnloaded =
        new SR_TrapezoidProfile.Constraints(
            kElevatorMaxSpeedMetersPerSecond,
            kElevatorMaxAccelerationMetersPerSecondSquaredUnloaded);

    // Constraint for the motion profilied elevator controller (loaded mode)
    public static final SR_TrapezoidProfile.Constraints kElevatorControllerConstraintsLoaded =
        new SR_TrapezoidProfile.Constraints(
            kElevatorMaxSpeedMetersPerSecond, kSlideMaxAccelerationMetersPerSecondSquaredLoaded);
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

    public static final double kArmFreeSpeedMetersPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kArmGearRatio * kSprocketCircumferenceMeters;

    public static final double kArmRotationTolerance = 0.01;
    // ARM TRANSLATION CONSTANTS
    public static final double ksArmTranslationVolts = 0.55953;
    public static final double kgArmTranslationVolts = 0.18092;
    public static final double kvArmTranslationVoltSecondsPerMeter = 62.802;
    public static final double kaArmTranslationVoltSecondsSquaredPerMeter = 0.04;

    public static final double kArmMaxSpeedTranslationMetersPerSecond =
        kArmFreeSpeedMetersPerSecond * 0.5;
    public static final double kArmMaxAccelerationTranslationMetersPerSecondSquared =
        new ElevatorFeedforward(
                ksArmTranslationVolts,
                kgArmTranslationVolts,
                kvArmTranslationVoltSecondsPerMeter,
                kaArmTranslationVoltSecondsSquaredPerMeter)
            .maxAchievableAcceleration(12.0, kArmMaxSpeedTranslationMetersPerSecond);

    // Constraint for the motion profilied arm rotation controller
    public static final SR_TrapezoidProfile.Constraints kArmControllerConstraintsTranslation =
        new SR_TrapezoidProfile.Constraints(
            kArmMaxSpeedTranslationMetersPerSecond,
            kArmMaxAccelerationTranslationMetersPerSecondSquared);

    public static final double kPArmTranslationVoltsPerMeter = 400;
    public static final double kDArmTranslationVoltSecondsPerMeter = 0.275;
    public static final double ksArmUnloadedVolts = 0.6;
    public static final double kgArmUnloadedVolts = 0.02;
    public static final double kvArmUnloadedVoltSecondsPerMeter = 55.315;
    public static final double kaArmUnloadedVoltSecondsSquaredPerMeter = 0.01;

    public static final double kArmTranslationToleranceMeters = 0.02;
    public static final double kArmTranslationVelocityToleranceMetersPerSecond = 0.05;
  }
}
