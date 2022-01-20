package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Drivetrain extends SubsystemBase implements Loggable {
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final PigeonIMU m_pigeon;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  @Log private double m_desiredRotation;
  @Log private double m_desiredXSpeed;
  @Log private double m_desiredYSpeed;

  @Log(name = "Drive Neutral")
  private SendableChooser<NeutralMode> m_driveNeutralChooser = new SendableChooser<NeutralMode>();

  @Log(name = "Turning Neutral")
  private SendableChooser<NeutralMode> m_turningNeutralChooser = new SendableChooser<NeutralMode>();

  private final Field2d m_field = new Field2d();

  private PhotonCamera m_camera;

  public Drivetrain(PhotonCamera camera) {
    m_frontLeftModule =
        new SwerveModule(
            kFLDriveTalonPort, kFLTurningTalonPort, kFLCANCoderPort, kFLCANCoderZero, "FL");
    m_frontRightModule =
        new SwerveModule(
            kFRDriveTalonPort, kFRTurningTalonPort, kFRCANCoderPort, kFRCANCoderZero, "FR");
    m_backLeftModule =
        new SwerveModule(
            kBLDriveTalonPort, kBLTurningTalonPort, kBLCANCoderPort, kBLCANCoderZero, "BL");
    m_backRightModule =
        new SwerveModule(
            kBRDriveTalonPort, kBRTurningTalonPort, kBRCANCoderPort, kBRCANCoderZero, "BR");

    m_pigeon = new PigeonIMU(kPigeonPort);

    Pose2d initialPoseMeters = new Pose2d(new Translation2d(0, 0), new Rotation2d());

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            this.getHeading(),
            initialPoseMeters,
            kDriveKinematics,
            new MatBuilder<>(Nat.N3(), Nat.N1())
                .fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N1(), Nat.N1())
                .fill(0.005), // Local measurement standard deviations. encoder and gyro
            new MatBuilder<>(Nat.N3(), Nat.N1())
                .fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    m_camera = camera;

    m_driveNeutralChooser.setDefaultOption("Brake", NeutralMode.Brake);
    m_driveNeutralChooser.addOption("Coast", NeutralMode.Coast);
    setDriveNeutralMode(m_driveNeutralChooser.getSelected());

    m_turningNeutralChooser.setDefaultOption("Brake", NeutralMode.Brake);
    m_turningNeutralChooser.addOption("Coast", NeutralMode.Coast);
    setDriveNeutralMode(m_turningNeutralChooser.getSelected());

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Drives the robot at given speeds and rotation. (Used in teleop)
   *
   * @param xSpeed desired forward velocity in meters per second
   * @param ySpeed desired sideways (strafe) velocity in meters per second
   * @param rot desired angular velocity in radians per second
   * @param fieldRelative whether the robot should drive field-relative or not
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kTeleopMaxSpeedMetersPerSecond);

    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backLeftModule.setDesiredState(states[2]);
    m_backRightModule.setDesiredState(states[3]);

    this.m_desiredRotation = rot;
    this.m_desiredXSpeed = xSpeed;
    this.m_desiredYSpeed = ySpeed;
  }

  /**
   * Directly sets module states. Used for autonomous driving.
   *
   * @param desiredStates array of swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kAutoMaxSpeedMetersPerSecond);
    m_frontLeftModule.setDesiredState(desiredStates[0]);
    m_frontRightModule.setDesiredState(desiredStates[1]);
    m_backLeftModule.setDesiredState(desiredStates[2]);
    m_backRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * The current pose of the robot.
   *
   * @return the pose as a Pose2d.
   */
  public Pose2d getPoseMeters() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets odometry to the specified pose.
   *
   * @param pose the pose to reset to
   */
  public void resetPoseEstimator(Pose2d pose) {
    m_poseEstimator.resetPosition(pose, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
  }

  @Log(name = "Pigeon Heading")
  public double getHeadingDegrees() {
    return m_pigeon.getFusedHeading();
  }

  /**
   * Returns the current heading reading from the Pigeon.
   *
   * @return
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  /** Sets the pigeon's current heading to zero. */
  public void zeroHeading() {
    m_pigeon.setFusedHeading(0);
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

  /** Returns the Field2d object. */
  public Field2d getField2d() {
    return m_field;
  }

  /**
   * Runs every 20 ms. Updates odometry based on encoder and gyro readings. Updates Field object
   * (Glass widget) based on odometry. Sets neutral modes to selected.
   */
  @Override
  public void periodic() {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_pigeon.getFusedHeading()),
        m_frontLeftModule.getState(),
        m_backRightModule.getState(),
        m_frontRightModule.getState(),
        m_backRightModule.getState());

    var result = m_camera.getLatestResult();

    SmartDashboard.putBoolean("Camera Has Target", result.hasTargets());

    if (result.hasTargets()) {
      Pose2d cameraEstimatedPose =
          PhotonUtils.estimateFieldToRobot(
              kCameraHeightMeters,
              kTargetHeightMeters,
              kCameraPitchRadians,
              Math.toRadians(result.getBestTarget().getPitch()),
              Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
              this.getHeading(),
              kFieldToTargetMeters,
              kCameraToRobotMeters);

      double imageCaptureTime = Timer.getFPGATimestamp() - result.getLatencyMillis();

      // System.out.println("cam estimated: " + cameraEstimatedPose.toString());
      m_poseEstimator.addVisionMeasurement(cameraEstimatedPose, imageCaptureTime);
    }

    // System.out.println("pose: " + getPoseMeters().toString());
    m_field.setRobotPose(getPoseMeters());

    setDriveNeutralMode(m_driveNeutralChooser.getSelected());
    setTurningNeutralMode(m_turningNeutralChooser.getSelected());
  }
}
