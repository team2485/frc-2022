package frc.util.photonVision;

import java.util.List;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.Pair;

public class PhotonPoseEstimator {
  public enum PoseStrategy {
    // pose with lowest ambiguity
    LOWEST_AMBIGUITY,

    // pose closest to camera height
    CLOSEST_TO_CAMERA_HEIGHT,

    // pose closest to set reference position
    CLOSEST_TO_REFERENCE_POSE,

    // pose closest to last pose calculated
    CLOSEST_TO_LAST_POSE,

    // pose determined by average of best targets
    AVERAGE_BEST_TARGETS
  }

  private AprilTagFieldLayout fieldTags;
  private PoseStrategy strategy;
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  private Pose3d lastPose;
  private Pose3d referencePose;
  private final Set<Integer> reportedErrors = new HashSet<>();

  /**
   * Create a new PhotonPoseEstimator.
   *
   * @param fieldTags     A WPILib {@link AprilTagFieldLayout} linking AprilTag
   *                      IDs to Pose3d objects
   *                      with respect to the FIRST field using the <a
   *                      href=
   *                      "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
   *                      Coordinate System</a>.
   * @param strategy      The strategy it should use to determine the best pose.
   * @param camera        PhotonCameras and
   * @param robotToCamera Transform3d from the center of the robot to the camera
   *                      mount positions
   *                      (ie, robot ➔ camera) in the <a
   *                      href=
   *                      "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *                      Coordinate System</a>.
   */
  public PhotonPoseEstimator(
      AprilTagFieldLayout fieldTags,
      PoseStrategy strategy,
      PhotonCamera camera,
      Transform3d robotToCamera) {
    this.fieldTags = fieldTags;
    this.strategy = strategy;
    this.camera = camera;
    this.robotToCamera = robotToCamera;
  }

  /**
   * Get the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @return the AprilTagFieldLayout
   */
  public AprilTagFieldLayout getFieldTags() {
    return fieldTags;
  }

  /**
   * Set the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @param fieldTags the AprilTagFieldLayout
   */
  public void setFieldTags(AprilTagFieldLayout fieldTags) {
    this.fieldTags = fieldTags;
  }

  /**
   * Get the Position Estimation Strategy being used by the Position Estimator.
   *
   * @return the strategy
   */
  public PoseStrategy getStrategy() {
    return strategy;
  }

  /**
   * Set the Position Estimation Strategy used by the Position Estimator.
   *
   * @param strategy the strategy to set
   */
  public void setStrategy(PoseStrategy strategy) {
    this.strategy = strategy;
  }

  /**
   * Return the reference position that is being used by the estimator.
   *
   * @return the referencePose
   */
  public Pose3d getReferencePose() {
    return referencePose;
  }

  /**
   * Update the stored reference pose for use when using the
   * <b>CLOSEST_TO_REFERENCE_POSE</b>
   * strategy.
   *
   * @param referencePose the referencePose to set
   */
  public void setReferencePose(Pose3d referencePose) {
    this.referencePose = referencePose;
  }

  /**
   * Update the stored reference pose for use when using the
   * <b>CLOSEST_TO_REFERENCE_POSE</b>
   * strategy.
   *
   * @param referencePose the referencePose to set
   */
  public void setReferencePose(Pose2d referencePose) {
    this.referencePose = new Pose3d(referencePose);
  }

  /**
   * Update the stored last pose. Useful for setting the initial estimate when
   * using the
   * <b>CLOSEST_TO_LAST_POSE</b> strategy.
   *
   * @param lastPose the lastPose to set
   */
  public void setLastPose(Pose3d lastPose) {
    this.lastPose = lastPose;
  }

  /**
   * Update the stored last pose. Useful for setting the initial estimate when
   * using the
   * <b>CLOSEST_TO_LAST_POSE</b> strategy.
   *
   * @param lastPose the lastPose to set
   */
  public void setLastPose(Pose2d lastPose) {
    setLastPose(new Pose3d(lastPose));
  }

  /**
   * Poll data from the configured cameras and update the estimated position of
   * the robot. Returns
   * empty if there are no cameras set or no targets were found from the cameras.
   *
   * @return an EstimatedRobotPose with an estimated pose, and information about
   *         the camera(s) and
   *         pipeline results used to create the estimate
   */
  public Optional<EstimatedRobotPose> update() {
    if (camera == null) {
      DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
      return Optional.empty();
    }

    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> estimatedPose;
    switch (strategy) {
      case LOWEST_AMBIGUITY:
        estimatedPose = lowestAmbiguityStrategy(result);
        break;
      case CLOSEST_TO_CAMERA_HEIGHT:
        estimatedPose = closestToCameraHeightStrategy(result);
        break;
      case CLOSEST_TO_LAST_POSE:
        estimatedPose = closestToReferencePoseStrategy(result, referencePose);
        break;
      case CLOSEST_TO_REFERENCE_POSE:
        estimatedPose = closestToReferencePoseStrategy(result, referencePose);
        break;
      case AVERAGE_BEST_TARGETS:
        estimatedPose = averageBestTargetsStrategy(result);
        break;
      default:
        DriverStation.reportError("[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
        return Optional.empty();
    }

    if (estimatedPose.isEmpty()) {
      lastPose = null;
    }

    return estimatedPose;
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity
   * from a List of
   * pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated
   *         timestamp of this
   *         estimation.
   */
  private Optional<EstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;

    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
      double targetPoseAmbiguity = target.getPoseAmbiguity();
      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity;
        lowestAmbiguityTarget = target;
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null)
      return Optional.empty();

    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

    if (targetPosition.isEmpty()) {
      reportFiducialPoseError(targetFiducialId);
      return Optional.empty();
    }

    return Optional.of(
        new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
            result.getTimestampSeconds()));
  }

  /**
   * Return the estimated position of the robot using the target with the lowest
   * delta height
   * difference between the estimated and actual height of the camera.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated
   *         timestamp of this
   *         estimation.
   */
  private Optional<EstimatedRobotPose> closestToCameraHeightStrategy(PhotonPipelineResult result) {
    double smallestHeightDifference = 10e9;
    EstimatedRobotPose closestHeightTarget = null;

    for (PhotonTrackedTarget target : result.targets) {
      int targetFiducialId = target.getFiducialId();

      // Don't report errors for non-fiducial targets. This could also be resolved by
      // adding -1 to
      // the initial HashSet.
      if (targetFiducialId == -1)
        continue;

      Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

      if (targetPosition.isEmpty()) {
        reportFiducialPoseError(target.getFiducialId());
        continue;
      }

      double alternateTransformDelta = Math.abs(
          robotToCamera.getZ()
              - targetPosition
                  .get()
                  .transformBy(target.getAlternateCameraToTarget().inverse())
                  .getZ());
      double bestTransformDelta = Math.abs(
          robotToCamera.getZ()
              - targetPosition
                  .get()
                  .transformBy(target.getBestCameraToTarget().inverse())
                  .getZ());

      if (alternateTransformDelta < smallestHeightDifference) {
        smallestHeightDifference = alternateTransformDelta;
        closestHeightTarget = new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(target.getAlternateCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
            result.getTimestampSeconds());
      }

      if (bestTransformDelta < smallestHeightDifference) {
        smallestHeightDifference = bestTransformDelta;
        closestHeightTarget = new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
            result.getTimestampSeconds());
      }
    }

    // Need to null check here in case none of the provided targets are fiducial.
    return Optional.ofNullable(closestHeightTarget);
  }

  /**
   * Return the estimated position of the robot using the target with the lowest
   * delta in the vector
   * magnitude between it and the reference pose.
   *
   * @param result        pipeline result
   * @param referencePose reference pose to check vector magnitude difference
   *                      against.
   * @return the estimated position of the robot in the FCS and the estimated
   *         timestamp of this
   *         estimation.
   */
  private Optional<EstimatedRobotPose> closestToReferencePoseStrategy(
      PhotonPipelineResult result, Pose3d referencePose) {
    if (referencePose == null) {
      DriverStation.reportError(
          "[PhotonPoseEstimator] Tried to use reference pose strategy without setting the reference!",
          false);
      return Optional.empty();
    }

    double smallestPoseDelta = 10e9;
    EstimatedRobotPose lowestDeltaPose = null;

    for (PhotonTrackedTarget target : result.targets) {
      int targetFiducialId = target.getFiducialId();

      // Don't report errors for non-fiducial targets. This could also be resolved by
      // adding -1 to
      // the initial HashSet.
      if (targetFiducialId == -1)
        continue;

      Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

      if (targetPosition.isEmpty()) {
        reportFiducialPoseError(targetFiducialId);
        continue;
      }

      Pose3d altTransformPosition = targetPosition
          .get()
          .transformBy(target.getAlternateCameraToTarget().inverse())
          .transformBy(robotToCamera.inverse());
      Pose3d bestTransformPosition = targetPosition
          .get()
          .transformBy(target.getBestCameraToTarget().inverse())
          .transformBy(robotToCamera.inverse());

      double altDifference = Math.abs(calculateDifference(referencePose, altTransformPosition));
      double bestDifference = Math.abs(calculateDifference(referencePose, bestTransformPosition));

      if (altDifference < smallestPoseDelta) {
        smallestPoseDelta = altDifference;
        lowestDeltaPose = new EstimatedRobotPose(altTransformPosition, result.getTimestampSeconds());
      }
      if (bestDifference < smallestPoseDelta) {
        smallestPoseDelta = bestDifference;
        lowestDeltaPose = new EstimatedRobotPose(bestTransformPosition, result.getTimestampSeconds());
      }
    }
    return Optional.ofNullable(lowestDeltaPose);
  }

  /**
   * Return the average of the best target poses using ambiguity as weight.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated
   *         timestamp of this
   *         estimation.
   */
  private Optional<EstimatedRobotPose> averageBestTargetsStrategy(PhotonPipelineResult result) {
    List<Pair<PhotonTrackedTarget, Pose3d>> estimatedRobotPoses = new ArrayList<>();
    double totalAmbiguity = 0;

    for (PhotonTrackedTarget target : result.targets) {
      int targetFiducialId = target.getFiducialId();

      // Don't report errors for non-fiducial targets. This could also be resolved by
      // adding -1 to
      // the initial HashSet.
      if (targetFiducialId == -1)
        continue;

      Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

      if (targetPosition.isEmpty()) {
        reportFiducialPoseError(targetFiducialId);
        continue;
      }

      double targetPoseAmbiguity = target.getPoseAmbiguity();

      // Pose ambiguity is 0, use that pose
      if (targetPoseAmbiguity == 0) {
        return Optional.of(
            new EstimatedRobotPose(
                targetPosition
                    .get()
                    .transformBy(target.getBestCameraToTarget().inverse())
                    .transformBy(robotToCamera.inverse()),
                result.getTimestampSeconds()));
      }

      totalAmbiguity += 1.0 / target.getPoseAmbiguity();

      estimatedRobotPoses.add(
          new Pair<>(
              target,
              targetPosition
                  .get()
                  .transformBy(target.getBestCameraToTarget().inverse())
                  .transformBy(robotToCamera.inverse())));
    }

    // Take the average

    Translation3d transform = new Translation3d();
    Rotation3d rotation = new Rotation3d();

    if (estimatedRobotPoses.isEmpty())
      return Optional.empty();

    for (Pair<PhotonTrackedTarget, Pose3d> pair : estimatedRobotPoses) {
      // Total ambiguity is non-zero confirmed because if it was zero, that pose was
      // returned.
      double weight = (1.0 / pair.getFirst().getPoseAmbiguity()) / totalAmbiguity;
      Pose3d estimatedPose = pair.getSecond();
      transform = transform.plus(estimatedPose.getTranslation().times(weight));
      rotation = rotation.plus(estimatedPose.getRotation().times(weight));
    }

    return Optional.of(
        new EstimatedRobotPose(new Pose3d(transform, rotation), result.getTimestampSeconds()));
  }

  /**
   * Difference is defined as the vector magnitude between the two poses
   *
   * @return The absolute "difference" (>=0) between two Pose3ds.
   */
  private double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
  }

  private void reportFiducialPoseError(int fiducialId) {
    if (!reportedErrors.contains(fiducialId)) {
      DriverStation.reportError(
          "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
      reportedErrors.add(fiducialId);
    }
  }
}
