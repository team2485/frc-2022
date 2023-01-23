package frc.util;

import edu.wpi.first.math.geometry.Pose3d;

/** An estimated pose based on pipeline result */
public class EstimatedRobotPose {
  /** The estimated pose */
  public final Pose3d estimatedPose;

  /** The estimated time the frame used to derive the robot pose was taken */
  public final double timestampSeconds;

  /**
   * Constructs an EstimatedRobotPose
   *
   * @param estimatedPose    estimated pose
   * @param timestampSeconds timestamp of the estimate
   */
  public EstimatedRobotPose(Pose3d estimatedPose, double timestampSeconds) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
  }
}
