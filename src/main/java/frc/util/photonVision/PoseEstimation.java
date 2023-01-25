package frc.util.photonVision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class PoseEstimation extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final Drivetrain m_drivetrain;

  private AprilTagFieldLayout aprilTagFieldLayout;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  // private static final Matrix<N1, N1> localMeasurementStdDevs =
  // VecBuilder.fill(Units.degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final Field2d field2d = new Field2d();

  private PhotonPipelineResult previousPipelineResult = null;

  public PoseEstimation(PhotonCamera camera, Drivetrain drivetrain) {
    this.m_camera = camera;
    this.m_drivetrain = drivetrain;

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, m_drivetrain.getYaw(),
        m_drivetrain.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

    List<AprilTag> aprilTags = new ArrayList<>();

    // instantiate april tag field positions
    aprilTags.add(new AprilTag(1, new Pose3d(VisionConstants.kRedSideAbsoluteXPos, -2.93659,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(2, new Pose3d(VisionConstants.kRedSideAbsoluteXPos, -1.26019,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(3, new Pose3d(VisionConstants.kRedSideAbsoluteXPos, .41621,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(4, new Pose3d(VisionConstants.kRedDriverAbsoluteXPos, 2.74161,
        VisionConstants.kDriverAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(5, new Pose3d(VisionConstants.kBlueDriverAbsoluteXPos, 2.74161,
        VisionConstants.kDriverAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(6, new Pose3d(VisionConstants.kBlueSideAbsoluteXPos, .41621,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(7, new Pose3d(VisionConstants.kBlueSideAbsoluteXPos, -1.25019,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));
    aprilTags.add(new AprilTag(8, new Pose3d(VisionConstants.kBlueSideAbsoluteXPos, -2.93659,
        VisionConstants.kScoringAbsoluteZPos, new Rotation3d(0, 0, 180))));

    aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, 0, 0);

    tab.addString("Pose (X, Y)", this::getFormattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);
  }

  @Override
  public void periodic() {
    // update pose estimator with visible targets
    var result = m_camera.getLatestResult();
    if (!result.equals(previousPipelineResult) && result.hasTargets()) {
      previousPipelineResult = result;
      double imageCaptureTime = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000d);

      for (PhotonTrackedTarget target : result.getTargets()) {
        var fiducialID = target.getFiducialId();
        if (fiducialID >= 0 && fiducialID < aprilTagFieldLayout.getTags().size()) {
          var targetPose = aprilTagFieldLayout.getTagPose(fiducialID);

          Transform3d cameraToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
              cameraToTarget.getTranslation().toTranslation2d(),
              cameraToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

          // TODO: possible failure point
          Pose2d camPose = targetPose.get().toPose2d().transformBy(transform.inverse());

          var visionMeasurement = camPose.transformBy(
              new Transform2d(VisionConstants.kCameraToRobot.getTranslation().toTranslation2d(), new Rotation2d()));

          m_poseEstimator.addVisionMeasurement(visionMeasurement, imageCaptureTime);
        }
      }
    }

    // update pose estimator with drivetrain sensors
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drivetrain.getYaw(), m_drivetrain.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", Units.metersToInches(pose.getX()), Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    m_drivetrain.zeroGyro();
    m_poseEstimator.resetPosition(m_drivetrain.getYaw(), m_drivetrain.getModulePositions(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    m_drivetrain.zeroGyro();
    m_poseEstimator.resetPosition(
        m_drivetrain.getYaw(), m_drivetrain.getModulePositions(), new Pose2d());
  }
}
