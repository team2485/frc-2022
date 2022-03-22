package frc.robot.subsystems.drive;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.TimestampedTranslation2d;
import frc.team2485.WarlordsLib.PoseHistory;
import frc.team2485.WarlordsLib.sendableRichness.SR_PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Optional;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase implements Loggable {
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final WPI_Pigeon2 m_pigeon;

  private final SwerveDriveOdometry m_odometry;
  // private final SwerveDriveOdometry m_odometryWithoutVision;

  private final SlewRateLimiter m_xAccelLimiterTeleop =
      new SlewRateLimiter(kTeleopMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter m_yAccelLimiterTeleop =
      new SlewRateLimiter(kTeleopMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter m_angAccelLimiterTeleop =
      new SlewRateLimiter(kTeleopMaxAngularAccelerationRadiansPerSecondSquared);

  private PoseHistory m_poseHistory = new PoseHistory(kPoseHistoryCapacity);
  private Translation2d m_velocity = new Translation2d();
  private boolean m_resetOnVision = false;
  @Log private double m_desiredRotation;
  @Log private double m_desiredXSpeed;
  @Log private double m_desiredYSpeed;

  @Log private boolean m_pushable;

  // @Log(name = "Drive Neutral")
  private SendableChooser<NeutralMode> m_driveNeutralChooser = new SendableChooser<NeutralMode>();

  // @Log(name = "Turning Neutral")
  private SendableChooser<NeutralMode> m_turningNeutralChooser = new SendableChooser<NeutralMode>();

  public final Field2d m_field = new Field2d();

  @Log(name = "Angle PID (hub tracking)")
  private final SR_PIDController m_rotationController = new SR_PIDController(kPRotation, 0, 0);

  private Supplier<Rotation2d> m_turretAngle;

  public Drivetrain(Supplier<Rotation2d> turretAngle) {
    m_frontLeftModule =
        new SwerveModule(
            kFLDriveTalonPort,
            kFLTurningTalonPort,
            kFLCANCoderPort,
            kFLCANCoderZero,
            kDriveInverted,
            "FL");

    m_frontRightModule =
        new SwerveModule(
            kFRDriveTalonPort,
            kFRTurningTalonPort,
            kFRCANCoderPort,
            kFRCANCoderZero,
            kDriveInverted,
            "FR");

    m_backLeftModule =
        new SwerveModule(
            kBLDriveTalonPort,
            kBLTurningTalonPort,
            kBLCANCoderPort,
            kBLCANCoderZero,
            kDriveInverted,
            "BL");
    m_backRightModule =
        new SwerveModule(
            kBRDriveTalonPort,
            kBRTurningTalonPort,
            kBRCANCoderPort,
            kBRCANCoderZero,
            kDriveInverted,
            "BR");

    m_pigeon = new WPI_Pigeon2(kPigeonPort);

    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(m_pigeon.getYaw()));

    // m_odometryWithoutVision =
    //     new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(m_pigeon.getYaw()));
    m_odometry.resetPosition(
        new Pose2d(new Translation2d(5, 3.9), new Rotation2d(0)),
        Rotation2d.fromDegrees(m_pigeon.getYaw()));

    // m_odometryWithoutVision.resetPosition(
    //     new Pose2d(
    //         new Translation2d(kRobotBumperLengthMeters / 2, kRobotBumperWidthMeters / 2),
    //         new Rotation2d(0)),
    //     Rotation2d.fromDegrees(m_pigeon.getYaw()));
    // m_odometryWithoutVision.resetPosition(
    //     new Pose2d(new Translation2d(0, 4.1148), new Rotation2d(0)),
    //     Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));

    // m_driveNeutralChooser.setDefaultOption("Brake", NeutralMode.Brake);
    // m_driveNeutralChooser.addOption("Coast", NeutralMode.Coast);
    // setDriveNeutralMode(m_driveNeutralChooser.getSelected());

    // m_turningNeutralChooser.setDefaultOption("Brake", NeutralMode.Brake);
    // m_turningNeutralChooser.addOption("Coast", NeutralMode.Coast);
    // setDriveNeutralMode(m_turningNeutralChooser.getSelected());

    m_rotationController.setTolerance(kRotationTolerance);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_turretAngle = turretAngle;
    SmartDashboard.putData("Field", m_field);
    // this.zeroHeading();

  }

  /**
   * Drives the robot at given speeds and rotation. (Used in teleop)
   *
   * @param xVelocity desired forward velocity in meters per second
   * @param yVelocity desired sideways (strafe) velocity in meters per second
   * @param angularVelocity desired angular velocity in radians per second
   * @param fieldRelative whether the robot should drive field-relative or not
   */
  public void drive(
      double xVelocity, double yVelocity, double angularVelocity, boolean fieldRelative) {
    xVelocity = m_xAccelLimiterTeleop.calculate(xVelocity);
    yVelocity = m_yAccelLimiterTeleop.calculate(yVelocity);
    angularVelocity = m_angAccelLimiterTeleop.calculate(angularVelocity);

    // if not being fed a speed, set all wheels pointing toward center to minimize pushability
    if (xVelocity == 0 && yVelocity == 0 && angularVelocity == 0) {
      if (m_pushable) {
        m_frontLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_frontRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_backLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_backRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
      } else {
        m_frontLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
        m_frontRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        m_backLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        m_backRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
      }

    } else {
      SwerveModuleState[] states =
          kDriveKinematics.toSwerveModuleStates(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xVelocity, yVelocity, angularVelocity, this.getHeading())
                  : new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
      SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);

      m_frontLeftModule.setDesiredState(states[0], false);
      m_frontRightModule.setDesiredState(states[1], false);
      m_backLeftModule.setDesiredState(states[2], false);
      m_backRightModule.setDesiredState(states[3], false);

      this.m_desiredRotation = angularVelocity;
      this.m_desiredXSpeed = xVelocity;
      this.m_desiredYSpeed = yVelocity;
    }
  }

  /**
   * Drives the robot at given speed and rotation position (used for hub facing drive).
   *
   * @param xVelocity desired forward velocity in meters per second
   * @param yVelocity desired sideways (strafe) velocity in meters per second
   * @param rotation desired rotation
   * @param fieldRelative whether the robot should drive field-relative or not
   */
  public void driveWithRotationPosition(
      double xVelocity, double yVelocity, double desiredRotation, boolean fieldRelative) {

    xVelocity = m_xAccelLimiterTeleop.calculate(xVelocity);
    yVelocity = m_yAccelLimiterTeleop.calculate(yVelocity);

    double angularVelocity = 0;
    if (Math.abs(desiredRotation % (2 * Math.PI) - this.getHeadingRadians() % (2 * Math.PI))
        > kRotationTolerance) {
      angularVelocity = m_rotationController.calculate(this.getHeadingRadians(), desiredRotation);
    }

    // if not being fed a speed, set all wheels pointing toward center to minimize pushability
    if (xVelocity == 0 && yVelocity == 0 && angularVelocity == 0) {
      if (m_pushable) {
        m_frontLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_frontRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_backLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
        m_backRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
      } else {
        m_frontLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
        m_frontRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        m_backLeftModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
        m_backRightModule.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
      }
    } else {
      SwerveModuleState[] states =
          kDriveKinematics.toSwerveModuleStates(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xVelocity,
                      yVelocity,
                      angularVelocity,
                      Rotation2d.fromDegrees(m_pigeon.getYaw()))
                  : new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
      SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);

      m_frontLeftModule.setDesiredState(states[0], false);
      m_frontRightModule.setDesiredState(states[1], false);
      m_backLeftModule.setDesiredState(states[2], false);
      m_backRightModule.setDesiredState(states[3], false);
    }
    this.m_desiredRotation = angularVelocity;
    this.m_desiredXSpeed = xVelocity;
    this.m_desiredYSpeed = yVelocity;
  }

  /**
   * Directly sets module states. Used for autonomous driving.
   *
   * @param desiredStates array of swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kAutoMaxSpeedMetersPerSecond);
    m_frontLeftModule.setDesiredState(desiredStates[0], false);
    m_frontRightModule.setDesiredState(desiredStates[1], false);
    m_backLeftModule.setDesiredState(desiredStates[2], false);
    m_backRightModule.setDesiredState(desiredStates[3], false);
  }

  /**
   * The current pose of the robot.
   *
   * @return the pose as a Pose2d.
   */
  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public Translation2d getFieldRelativeVelocityMetersPerSecond() {
    return m_velocity;
  }

  public double getRobotRelativeForwardVelocityMetersPerSecond() {
    return m_velocity.getX() * this.getHeadingRadians();
  }

  @Log(name = "Hub to turret center")
  public double getHubToTurretCenterDistanceMeters() {
    return getTurretCenterPoseMeters().getTranslation().getDistance(kHubCenterTranslation);
  }

  public Pose2d getTurretCenterPoseMeters() {
    return getPoseMeters().plus(kRobotToTurretCenterMeters);
    // .plus(
    // new Transform2d(new Translation2d(0, 0), this.getHeading())));
  }

  /**
   * Resets odometry to the specified pose.
   *
   * @param pose the pose to reset to
   */
  public void resetOdometry(Pose2d pose, boolean clearHistory) {
    if (clearHistory) {
      m_poseHistory = new PoseHistory(kPoseHistoryCapacity);
    }
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(m_pigeon.getYaw()));
  }

  // @Log(name = "Pigeon Heading")
  public double getPigeonHeadingDegrees() {
    return m_pigeon.getYaw();
  }

  /** Returns the current heading reading from the Pigeon. */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  @Log(name = "Heading Radians")
  public double getHeadingRadians() {
    return this.getHeading().getRadians();
  }

  /** Sets the pigeon's current heading to zero. */
  public void zeroHeading() {
    m_odometry.resetPosition(
        new Pose2d(this.getPoseMeters().getTranslation(), new Rotation2d(0)),
        Rotation2d.fromDegrees(m_pigeon.getYaw()));
  }

  /** Returns the current angular velocity in radians per second */
  public double getAngularVelocityRadiansPerSecond() {
    return Math.toRadians(-m_pigeon.getRate());
  }

  /**
   * Configures neutral modes for each drive motor.
   *
   * @param mode NeutralMode.Coast or NeutralMode.Brake
   */
  public void setDriveNeutralMode(NeutralMode mode) {
    m_frontLeftModule.setDriveNeutralMode(mode);
    m_frontRightModule.setDriveNeutralMode(mode);
    m_backLeftModule.setDriveNeutralMode(mode);
    m_backRightModule.setDriveNeutralMode(mode);
  }

  /**
   * Configures neutral modes for each turning motor.
   *
   * @param mode NeutralMode.Coast or NeutralMode.Brake
   */
  public void setTurningNeutralMode(NeutralMode mode) {
    m_frontLeftModule.setTurningNeutralMode(mode);
    m_frontRightModule.setTurningNeutralMode(mode);
    m_backLeftModule.setTurningNeutralMode(mode);
    m_backRightModule.setTurningNeutralMode(mode);
  }

  @Config.ToggleSwitch(name = "Set pushable")
  public void setPushable(boolean pushable) {
    m_pushable = pushable;
    if (m_pushable) {
      this.setDriveNeutralMode(NeutralMode.Coast);
      this.setTurningNeutralMode(NeutralMode.Brake);
    } else {
      this.setDriveNeutralMode(NeutralMode.Brake);
      this.setTurningNeutralMode(NeutralMode.Brake);
    }
  }

  /** Returns the Field2d object. */
  public Field2d getField2d() {
    return m_field;
  }

  public void addVisionMeasurement(TimestampedTranslation2d data) {
    Optional<Pose2d> historicalFieldToTarget = m_poseHistory.get(data.timestamp);

    if (historicalFieldToTarget.isPresent()) {
      // Calculate new robot pose

      Rotation2d robotRotation = historicalFieldToTarget.get().getRotation();
      Rotation2d cameraRotation =
          robotRotation.rotateBy(
              kRobotToTurretCenterMeters.getRotation().plus(m_turretAngle.get()));
      Transform2d fieldToTargetRotated = new Transform2d(kHubCenterTranslation, cameraRotation);
      Transform2d fieldToCamera =
          fieldToTargetRotated.plus(
              new Transform2d(data.translation.unaryMinus(), new Rotation2d()));

      Transform2d visionFieldToTargetTransform =
          fieldToCamera.plus(
              kRobotToTurretCenterMeters
                  .plus(new Transform2d(kTurretCentertoCameraMeters, m_turretAngle.get()))
                  .inverse());

      Pose2d visionFieldToTarget =
          new Pose2d(
              visionFieldToTargetTransform.getTranslation(),
              visionFieldToTargetTransform.getRotation());

      // Calculate percent weight to give to vision
      // Calculate current percentage of max angular velocity
      double angularErrorScale =
          MathUtil.clamp(
              Math.abs(this.getAngularVelocityRadiansPerSecond())
                  / kVisionMaxAngularVelocityRadians,
              0,
              1);

      // find weight to give vision
      double visionWeight = 1 - Math.pow(1 - kVisionWeightPerSec, 1 / kVisionNominalFramerate);

      // Scale weight to be 0 when angular velocity is at max
      visionWeight *= 1 - angularErrorScale;

      // Reset pose
      Pose2d currentFieldToTarget = getPoseMeters();
      Translation2d fieldToVisionField =
          new Translation2d(
              visionFieldToTarget.getX() - historicalFieldToTarget.get().getX(),
              visionFieldToTarget.getY() - historicalFieldToTarget.get().getY());

      Pose2d visionLatencyCompFieldToTarget =
          new Pose2d(
              currentFieldToTarget.getX() + fieldToVisionField.getX(),
              currentFieldToTarget.getY() + fieldToVisionField.getY(),
              currentFieldToTarget.getRotation());

      m_field.getObject("Vision").setPose(visionLatencyCompFieldToTarget);

      if (m_resetOnVision) {
        resetOdometry(
            new Pose2d(
                visionFieldToTarget.getX(),
                visionFieldToTarget.getY(),
                currentFieldToTarget.getRotation()),
            true);

        m_resetOnVision = false;
      } else {
        resetOdometry(
            new Pose2d(
                currentFieldToTarget.getX() * (1 - visionWeight)
                    + visionLatencyCompFieldToTarget.getX() * visionWeight,
                currentFieldToTarget.getY() * (1 - visionWeight)
                    + visionLatencyCompFieldToTarget.getY() * visionWeight,
                currentFieldToTarget.getRotation()),
            false);
      }
    }
  }

  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d transformFromTranslation(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }
  /**
   * Runs every 20 ms. Updates odometry based on encoder and gyro readings. Updates Field object
   * (Glass widget) based on odometry. Sets neutral modes to selected.
   */
  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_frontLeftModule.getState(),
        m_backRightModule.getState(),
        m_frontRightModule.getState(),
        m_backRightModule.getState());

    // m_odometryWithoutVision.update(
    //     Rotation2d.fromDegrees(m_pigeon.getYaw()),
    //     m_frontLeftModule.getState(),
    //     m_backRightModule.getState(),
    //     m_frontRightModule.getState(),
    //     m_backRightModule.getState());

    m_field.getObject("Turret").setPose(this.getTurretCenterPoseMeters());

    Pose2d robotPose = m_odometry.getPoseMeters();
    // Pose2d lastPose;
    // try {
    //   lastPose = m_poseHistory.getLatest().get().getPose();
    // } catch (NoSuchElementException e) {
    //   lastPose = robotPose;
    // }
    // m_velocity = robotPose.getTranslation().minus(lastPose.getTranslation());
    m_poseHistory.insert(Timer.getFPGATimestamp(), robotPose);

    // System.out.println("pose: " + getPoseMeters().toString());
    m_field.setRobotPose(getPoseMeters());

    // Update brake/coast mode
    // setDriveNeutralMode(m_driveNeutralChooser.getSelected());
    // setTurningNeutralMode(m_turningNeutralChooser.getSelected());
  }
}
