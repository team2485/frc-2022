// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final double kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;
  public static final double kSecondsPer100Ms = 0.1;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

  // meters and shot parameters (radians and rps)
  public static final TreeMap<Double, ShotParameter> kShootingMap =
      new TreeMap<>(
          Map.ofEntries(
              entry(2.76, new ShotParameter(105, 0, 0)),
              entry(3.2, new ShotParameter(120, 0.455, 0)),
              entry(3.6, new ShotParameter(123, 0.475, 0))));

  // 5 ft front bumper: 60 0.8

  public static final double kShootingFenderSetpointShooter = 24;
  public static final double kShootingFenderSetpointTangentialRatio = 0.55;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.1;
    public static final double kDriverLeftXDeadband = 0.15;
    public static final double kDriverLeftYDeadband = 0.15;

    public static final double kTriggerThreshold = 0.1;
  }

  public static final class AutoConstants {
    public static final double kAutoMaxSpeedMetersPerSecond = 1.5;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        1.5 / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 1 * Math.PI;

    public static final double kPAutoXController = 5;
    public static final double kIAutoXController = 0.05;
    public static final double kDAutoXController = 0.2;
    public static final double kPAutoYController = 5;
    public static final double kIAutoYController = 0.05;
    public static final double kDAutoYController = 0.2;

    public static final double kAutoXYIntegratorMaxMetersPerSecond = 0.5;
    public static final double kPAutoThetaController = 8;
    public static final double kIAutoThetaController = 0.1;
    public static final double kAutoThetaIntegratorMaxRadiansPerSecond = 0.2;
    public static final double kDAutoThetaController = 0.1;

    // Constraint for the motion profilied robot angle controller
    public static final SR_TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new SR_TrapezoidProfile.Constraints(
            kAutoMaxAngularSpeedRadiansPerSecond,
            kAutoMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ModuleConstants {
    // Drive control constants
    public static final double kDriveSupplyCurrentLimitAmps = 35;
    public static final double kDriveStatorCurrentLimitAmps = 60;
    public static final double kDriveStatorCurrentThresholdTimeSecs = 0.1;

    //// Drive mechanism/encoder constants
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = 0.1016 * Math.PI;
    public static final double kDriveGearRatio = 6.86; // motor turns per wheel turns
    public static final double kDriveDistMetersPerMotorRev =
        kWheelCircumferenceMeters / kDriveGearRatio;
    public static final double kDriveDistMetersPerPulse =
        kDriveDistMetersPerMotorRev / kFalconSensorUnitsPerRotation;
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
    public static final double ksDriveVolts = 0.51019;
    public static final double kvDriveVoltSecondsPerMeter = 2.2644;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.1;

    public static final double ksDriveVoltsBackLeft = 0.5;

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.1;
    // Turning control constants
    public static final double kTurningSupplyCurrentLimitAmps = 20;
    public static final double kTurningStatorCurrentLimitAmps = 60;
    public static final double kTurningStatorCurrentThresholdTimeSecs = 0.1;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse =
        kTurningRadiansPerMotorRev / kFalconSensorUnitsPerRotation;

    //// Turning feedforward constants
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerRadian = 0.20175;
    public static final double kaTurningVoltSecondsSquaredPerRadian = 0.0053;

    //// Turning PID constants
    public static final double kPTurningOutputUnit100MsPerSensorUnit =
        1.5 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kDTurningOutputUnit100MsSquaredPerSensorUnit =
        0.2 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kFTurningOutputUnit100MsPerSensorUnit = 0.4 * 1023 / 8360;

    public static final double kTurningPositionToleranceSensorUnits =
        Units.degreesToRadians(2) * kFalconSensorUnitsPerRotation;
    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 8 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 64 * Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms =
        kModuleMaxSpeedTurningRadiansPerSecond / kTurningRadiansPerPulse * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared =
        kModuleMaxAccelerationTurningRadiansPerSecondSquared / kTurningRadiansPerPulse * 0.01;
  }

  public static final class DriveConstants {
    // Ports and zeros
    /** Zeros found with bevel gears facing right. Applied offset is the negative of the zero. */
    public static final int kPigeonPort = 9;

    public static final boolean kDriveInverted = true;
    public static final int kFLDriveTalonPort = 5;
    public static final int kFLTurningTalonPort = 6;
    public static final int kFLCANCoderPort = 12;
    // public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(56.0 - 1.66);
    public static final Rotation2d kFLCANCoderZero = Rotation2d.fromDegrees(47.96);

    public static final int kFRDriveTalonPort = 7;
    public static final int kFRTurningTalonPort = 8;
    public static final int kFRCANCoderPort = 13;
    // public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(-85.48 - 135);
    public static final Rotation2d kFRCANCoderZero = Rotation2d.fromDegrees(136.48 - 180);

    public static final int kBRDriveTalonPort = 1;
    public static final int kBRTurningTalonPort = 2;
    public static final int kBRCANCoderPort = 10;
    // public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(-3.6 - 5.18);
    public static final Rotation2d kBRCANCoderZero = Rotation2d.fromDegrees(170.89);

    public static final int kBLDriveTalonPort = 3;
    public static final int kBLTurningTalonPort = 4;
    public static final int kBLCANCoderPort = 11;
    // public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(-167.7 + 4.33 -
    // 3.06);
    public static final Rotation2d kBLCANCoderZero = Rotation2d.fromDegrees(-168.98);

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
        1.5 / kTurningRadiusMeters; // radians per second

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared = 1.5 * Math.PI;

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

    public static final double kVisionNominalFramerate = 45;

    public static final double kLensHeightMeters = 0.56896;
    public static final double kLensPitchRadians = Units.degreesToRadians(30);
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

    public static final double kExtraLatencySecs = 0;

    // idle behavior
    public static final double kTargetGraceSecs =
        0.5; // how long after target loss to wait for reaquire before turning to blink mode
    public static final double kBlinkPeriodSecs = 3.0;
    public static final double kBlinkLengthSecs = 0.5;

    public static final Transform2d kRobotToCameraMeters =
        new Transform2d(
            new Translation2d(0.321818, 0), // in meters
            new Rotation2d());

    // Vision pose estimation constants
    public static final double kVisionWeightPerSec =
        0.85; // After one second, what % of pose average should be vision (4% in weighted avg)

    public static final double kVisionMaxAngularVelocityRadians =
        Units.degreesToRadians(8.0); // Max angular velocity before vision data is rejected
  }

  public static final class IntakeConstants {
    public static final int kIntakeTalonPort = 20;
    public static final double kIntakeLoopTimeSeconds = 0.02;
    public static final int kIntakeSmartCurrentLimitAmps = 20;
    public static final int kIntakeImmediateCurrentLimitAmps = 25;

    public static final double kIntakeSupplyCurrentLimitAmps = 25;
    public static final double kIntakeSupplyCurrentThresholdAmps = 30;
    public static final double kIntakeSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kIntakeStatorCurrentLimitAmps = 40;
    public static final double kIntakeStatorCurrentThresholdAmps = 45;
    public static final double kIntakeStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kIntakeGearRatio = 2; // motor turns : output/full hood turns

    public static final double kIntakeFreeSpeedRotationsPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kIntakeGearRatio;

    public static final double kIntakeTopWheelDiameterMeters = 0.1016; // 4 in
    public static final double kIntakeBottomWheelDiameterMeters = 0.1524; // 6 in
    public static final double kIntakeBottomWheelCircumferenceMeters = 0.1524 * Math.PI;
    public static final double kIntakeDefaultSpeedRotationsPerSecond = 2;

    public static final double kSIntakeVolts = 0.75191;
    public static final double kVIntakeVoltSecondsPerMeter = 0.45447;
    public static final double kAIntakeVoltSecondsSquaredPerMeter = 0.026443;

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
    public static final int kIntakeArmSmartCurrentLimitAmps = 30;
    public static final int kIntakeArmImmediateCurrentLimitAmps = 35;

    public static final double kIntakeArmGearRatio = 125.0;
    public static final double kIntakeArmFreeSpeedRadiansPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kIntakeArmGearRatio * (2 * Math.PI);
    public static final double kIntakeArmRadiansPerMotorRev =
        1.0 / kIntakeArmGearRatio * 2 * Math.PI;

    public static final double kIntakeArmBottomPositionRadians =
        IDManager.getInstance().select(-0.204, -0.2618); // from horizontal
    public static final double kIntakeArmTopPositionRadians =
        IDManager.getInstance().select(2.019, 2.0071); // change later
    public static final double kIntakeArmTipPositionRadians =
        IDManager.getInstance().select(1.25, 1.6);

    public static final double kIntakeArmEncoderOffset =
        IDManager.getInstance()
            .select(3.28 + 0.038 - 0.2618 - 0.702 - 0.2418 - 0.03 - 0.13 + 0.0861, 5.44 - 0.2618);

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
    public static final double kIntakeArmPositionToleranceRadians = 0.02;
  }

  public static final class IndexerConstants {
    public static final int kIndexerTalonPort = 18;
    public static final double kIndexerLoopTimeSeconds = 0.02;
    public static final int kIndexerSmartCurrentLimitAmps = 25;
    public static final int kIndexerImmediateCurrentLimitAmps = 30;

    public static final double kIndexerSupplyCurrentLimitAmps = 25;
    public static final double kIndexerSupplyCurrentThresholdAmps = 30;
    public static final double kIndexerSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kIndexerStatorCurrentLimitAmps = 40;
    public static final double kIndexerStatorCurrentThresholdAmps = 45;
    public static final double kIndexerStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kIndexerGearRatio = 4; // motor turns : output/full hood turns

    public static final double kIndexerFreeSpeedRotationsPerSecond =
        kNeoFreeSpeedRotationsPerSecond / kIndexerGearRatio;

    public static final double kIndexerEntryWheelDiameterMeters = 0.0508; // 2 inches

    public static final double kIndexerIntakeSpeedRatio =
        IntakeConstants.kIntakeTopWheelDiameterMeters / kIndexerEntryWheelDiameterMeters;

    public static final double kIndexerDefaultSpeedRotationsPerSecond =
        kIndexerFreeSpeedRotationsPerSecond * 0.9;

    public static final double kSIndexerVolts = 0.69534;
    public static final double kVIndexerVoltSecondsPerMeter = 0.5;
    public static final double kAIndexerVoltSecondsSquaredPerMeter = 0.0015969;

    public static final double kIndexerVelocityToleranceRotationsPerSecond = 1;
  }

  public static final class FeederConstants {
    public static final int kFeederTalonPort = 16;
    public static final double kFeederLoopTimeSeconds = 0.02;
    public static final int kFeederSmartCurrentLimitAmps = 15;
    public static final int kFeederImmediateCurrentLimitAmps = 20;

    public static final double kFeederSupplyCurrentLimitAmps = 25;
    public static final double kFeederSupplyCurrentThresholdAmps = 30;
    public static final double kFeederSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kFeederStatorCurrentLimitAmps = 40;
    public static final double kFeederStatorCurrentThresholdAmps = 45;
    public static final double kFeederStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kFeederGearRatio = 1; // motor turns : output/full hood turns

    public static final double kFeederFreeSpeedRotationsPerSecond =
        kNeo550FreeSpeedRotationsPerSecond / kFeederGearRatio;

    public static final double kFeederDefaultSpeedRotationsPerSecond =
        kFeederFreeSpeedRotationsPerSecond * 0.75;

    public static final double kFeederPulleyCircumferenceMeters = 0.0191008 * Math.PI;

    public static final double kFeederSurfaceFreeSpeedMetersPerSecond =
        kFeederFreeSpeedRotationsPerSecond * kFeederPulleyCircumferenceMeters;

    public static final double kFeederShooterSurfaceSpeedRatio =
        kFeederSurfaceFreeSpeedMetersPerSecond
            / ShooterConstants.kShooterSurfaceFreeSpeedMetersPerSecond;

    public static final double kSFeederVolts = 0.56782;
    public static final double kVFeederVoltSecondsPerMeter = 0.6591;
    public static final double kAFeederVoltSecondsSquaredPerMeter = 0.032787;

    public static final double kFeederVelocityToleranceRotationsPerSecond = 1;

    public static final int kFeederServoPort = 1;
    public static final double kServoDisengagedPosition = 0.43;
    public static final double kServoEngagedPosition = 0.19;
  }

  public static final class HoodConstants {
    public static final int kHoodSparkPort = 23;
    public static final double kHoodGearRatio = 225; // motor turns : output/full hood turns
    public static final double kHoodRadiansPerMotorRev = 2 * Math.PI / kHoodGearRatio;

    public static final double kHoodBottomPositionRadians = 0; // from horizontal
    public static final double kHoodTopPositionRadians = 0.2872;

    public static final int kHoodSmartCurrentLimitAmps = 10;
    public static final int kHoodImmediateCurrentLimitAmps = 10;

    // Hood characterization constants
    public static final double kSHoodVolts = 0.1;
    public static final double kGHoodVolts = 0.25;
    public static final double kVHoodVoltSecondsPerRadian = 1;
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
    public static final double kIHood = 3;
    public static final double kDHood = 0;
    public static final double kHoodControllerPositionTolerance = 0.005;
  }

  public static final class ShooterConstants {
    public static final int kShooterTalonPort1 = 17;
    public static final int kShooterTalonPort2 = 19;

    public static final double kShooterSupplyCurrentLimitAmps = 25;
    public static final double kShooterSupplyCurrentThresholdAmps = 30;
    public static final double kShooterSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kShooterStatorCurrentLimitAmps = 40;
    public static final double kShooterStatorCurrentThresholdAmps = 45;
    public static final double kShooterStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kKickerSupplyCurrentLimitAmps = 40;
    public static final double kKickerSupplyCurrentThresholdAmps = 45;
    public static final double kKickerSupplyCurrentThresholdTimeSecs =
        kShooterSupplyCurrentLimitAmps;
    public static final double kKickerStatorCurrentLimitAmps = 60;
    public static final double kKickerStatorCurrentThresholdAmps = 65;
    public static final double kKickerStatorCurrentThresholdTimeSecs =
        kShooterStatorCurrentThresholdTimeSecs;

    public static final double kShooterLoopTimeSeconds = 0.001;
    public static final double kKickerLoopTimeSeconds = kShooterLoopTimeSeconds;

    public static final double kShooterGearRatio = 1;
    public static final double kKickerGearRatio = 0.5;

    public static final double kShooterCircumferenceMeters =
        0.1524 * Math.PI; // 6 in diameter wheel
    public static final double kKickerCircumferenceMeters = 0.0508 * Math.PI; // 2 in diameter wheel

    public static final double kShooterFreeSpeedRotationsPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kShooterGearRatio;
    public static final double kKickerFreeSpeedRotationsPerSecond =
        kFalconFreeSpeedRotationsPerSecond / kKickerGearRatio;

    public static final double kShooterMaxSpeedRotationsPerSecond = 68; // empirical estimate
    public static final double kKickerMaxSpeedRotationsPerSecond = 180; // empirical estimate

    public static final double kShooterSurfaceFreeSpeedMetersPerSecond =
        kShooterFreeSpeedRotationsPerSecond * kShooterCircumferenceMeters;
    public static final double kKickerSurfaceFreeSpeedMetersPerSecond =
        kKickerFreeSpeedRotationsPerSecond * kKickerCircumferenceMeters;

    public static final double kSShooterVolts = 0.25;
    public static final double kVShooterVoltSecondsPerRotation = 0.133;
    public static final double kAShooterVoltSecondsSquaredPerRotation =
        IDManager.getInstance().select(0.05, 0.005);

    public static final double kSKickerVolts = 0.5;
    public static final double kVKickerVoltSecondsPerRotation = 0.1;
    public static final double kAKickerVoltSecondsSquaredPerRotation = 0.0019767;

    public static final double kFShooterOutputUnit100MsPerSensorUnit =
        kVShooterVoltSecondsPerRotation
            * kFalconOutputUnitsPerVolt
            / kShooterGearRatio
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;
    public static final double kFKickerOutputUnit100MsPerSensorUnit =
        kVKickerVoltSecondsPerRotation
            / kKickerGearRatio
            * kFalconOutputUnitsPerVolt
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kPShooterVoltSecondsPerRotation = 0.01; // 0.5
    public static final double kPShooterOutputUnit100MsPerSensorUnit =
        kPShooterVoltSecondsPerRotation
            * kFalconOutputUnitsPerVolt
            / kShooterGearRatio
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kPKickerVoltSecondsPerRotation = 0.1; // 0.5
    public static final double kPKickerOutputUnit100MsPerSensorUnit =
        kPKickerVoltSecondsPerRotation
            / kKickerGearRatio
            * kFalconOutputUnitsPerVolt
            / kSecondsPer100Ms
            / kFalconSensorUnitsPerRotation;

    public static final double kShooterControlVelocityToleranceRotationsPerSecond = 1;
    public static final double kShooterControlVelocityToleranceSensorUnitsPer100Ms =
        kShooterControlVelocityToleranceRotationsPerSecond
            * kSecondsPer100Ms
            * kFalconSensorUnitsPerRotation;

    public static final double kKickerControlVelocityToleranceRotationsPerSecond =
        kShooterControlVelocityToleranceRotationsPerSecond
            * (kKickerFreeSpeedRotationsPerSecond / kShooterFreeSpeedRotationsPerSecond);
    public static final double kKickerControlVelocityToleranceSensorUnitsPer100Ms =
        kKickerControlVelocityToleranceRotationsPerSecond
            * kSecondsPer100Ms
            * kFalconSensorUnitsPerRotation;

    public static final double kShooterFeedforwardScale = 0.86;
    public static final double kKickerFeedforwardScale = 0.88;

    public static final double kShooterFeedVelocityTolerance = 3;

    public static final double kDefaultTangentialVelocityRatio =
        1.0 / 3.0; // kicker tangential velocity / shooter tangential velocity
  }

  public static final class ClimbElevatorConstants {

    public static final double kElevatorControlLoopTimeSeconds = 0.01;
    // SLIDE CONSTANTS
    public static final int kElevatorTalonPort = 40;
    public static final double kElevatorSupplyCurrentLimitAmps = 25;
    public static final double kElevatorSupplyCurrentThresholdAmps = 30;
    public static final double kElevatorSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kElevatorStatorCurrentLimitAmps = 40;
    public static final double kElevatorStatorCurrentThresholdAmps = 45;
    public static final double kElevatorStatorCurrentThresholdTimeSecs = 0.05;

    public static final int kElevatorSlotSensorTopPort = 0; // dio
    public static final int kElevatorSlotSensorBottomPort = 1; // dio

    public static final double kElevatorTopStopPosition = Units.inchesToMeters(0);
    public static final double kElevatorBottomStopPosition = Units.inchesToMeters(-21.25);

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
        kElevatorDistancePerMotorRevMeters / kFalconSensorUnitsPerRotation;

    // Slide characterization constants: UNLOADED (not carrying robot)
    public static final double ksElevatorUnloadedVolts =
        IDManager.getInstance().select(0.62614, 0.70015);
    public static final double kgElevatorUnloadedVolts =
        IDManager.getInstance().select(0.0906651, 0.010378);
    public static final double kvElevatorUnloadedVoltSecondsPerMeter =
        IDManager.getInstance().select(33.80, 30.0);
    public static final double kaElevatorUnloadedVoltSecondsSquaredPerMeter =
        IDManager.getInstance().select(0.01, 0.01);

    // Slide characterization constants: LOADED ( carrying robot)
    public static final double ksElevatorLoadedVolts =
        IDManager.getInstance().select(0.64858, 0.44256);
    public static final double kgElevatorLoadedVolts =
        IDManager.getInstance()
            .select(
                -0.7,
                -0.50); // this is negative because gravity fights the downward motion when loaded
    // --
    // retracting the elevator moves the robot up.
    public static final double kvElevatorLoadedVoltSecondsPerMeter =
        IDManager.getInstance().select(42.0, 30.0);
    public static final double kaElevatorLoadedVoltSecondsSquaredPerMeter =
        IDManager.getInstance()
            .select(
                0.18, 0.18); // these are recalc gains -- the ka from sysid was lost in the noise

    public static final double kPElevatorUnloadedVoltsPerMeter = 100;
    public static final double kDElevatorUnloadedVoltSecondsPerMeter = 0.05;

    public static final double kPElevatorLoadedVoltsPerMeter = 280;
    public static final double kDElevatorLoadedVoltSecondsPerMeter = 10;

    public static final double kElevatorPositionToleranceMeters = 0.008;
    public static final double kElevatorVelocityToleranceMetersPerSecond = 0.01;

    public static final double kElevatorMaxSpeedMetersPerSecond =
        kElevatorFreeSpeedMetersPerSecond * 0.95;
    // Find maximum simultaneously achievable acceleration
    public static final double kElevatorMaxAccelerationMetersPerSecondSquaredUnloaded = 0.8;

    public static final double kSlideMaxAccelerationMetersPerSecondSquaredLoaded = 0.5;

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

    public static final double kArmGearingChange = 8.0 / 11.0;
    // ARM CONSTANTS
    public static final int kArmTalonPort = 41;
    public static final double kArmSupplyCurrentLimitAmps = 25;
    public static final double kArmSupplyCurrentThresholdAmps = 30;
    public static final double kArmSupplyCurrentThresholdTimeSecs = 0.1;
    public static final double kArmStatorCurrentLimitAmps = 25;
    public static final double kArmStatorCurrentThresholdAmps = 30;
    public static final double kArmStatorCurrentThresholdTimeSecs = 0.05;

    public static final double kArmStatorCurrentSpikeThresholdAmps = 30;
    public static final double kArmStatorCurrentSpikeDebounceTimeSeconds = 0.2;

    public static final double kArmGearRatio = 57.6 * (16.0 / 22.0); // motor turns/pinion turns
    public static final double kArmRotationsPerMotorRev = 1 / kArmGearRatio;
    public static final double kArmRotationsPerPulse =
        kArmRotationsPerMotorRev / kFalconSensorUnitsPerRotation;

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
    public static final double ksArmTranslationVolts =
        IDManager.getInstance().select(0.47533, 0.55953);
    public static final double kgArmTranslationVolts =
        IDManager.getInstance().select(0.25818, 0.18092);
    public static final double kvArmTranslationVoltSecondsPerMeter =
        IDManager.getInstance().select(63.445 * kArmGearingChange, 62.802);
    public static final double kaArmTranslationVoltSecondsSquaredPerMeter =
        IDManager.getInstance().select(0.07 * kArmGearingChange, 0.04);

    public static final double kArmMaxSpeedTranslationMetersPerSecond =
        kArmFreeSpeedMetersPerSecond * 0.9 / kArmGearingChange;
    public static final double kArmMaxAccelerationTranslationMetersPerSecondSquared =
        10 * kArmGearingChange;

    // Constraint for the motion profilied arm rotation controller
    public static final SR_TrapezoidProfile.Constraints kArmControllerConstraintsTranslation =
        new SR_TrapezoidProfile.Constraints(
            kArmMaxSpeedTranslationMetersPerSecond,
            kArmMaxAccelerationTranslationMetersPerSecondSquared);

    public static final double kPArmTranslationVoltsPerMeter = 400 * kArmGearingChange;
    public static final double kDArmTranslationVoltSecondsPerMeter = 0.275 * kArmGearingChange;
    public static final double kIArmTranslationVoltsPerMeter = 30;
    public static final double kArmIntegratorMaxVolts = 0.5;
    public static final double ksArmUnloadedVolts = 0.47;
    public static final double kgArmUnloadedVolts = 0.02;
    public static final double kvArmUnloadedVoltSecondsPerMeter = 63.445 * kArmGearingChange;
    public static final double kaArmUnloadedVoltSecondsSquaredPerMeter = 0.01 * kArmGearingChange;

    public static final double kArmTranslationToleranceMeters = 0.02;
    public static final double kArmTranslationVelocityToleranceMetersPerSecond =
        0.05 / kArmGearingChange;
  }
}
