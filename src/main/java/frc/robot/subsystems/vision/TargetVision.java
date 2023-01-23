package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.util.photonVision.EstimatedRobotPose;
import frc.util.photonVision.PhotonCamera;
import frc.util.photonVision.PhotonPoseEstimator;
import frc.util.photonVision.PhotonPoseEstimator.PoseStrategy;

public class TargetVision extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_photonPoseEstimator;

  private double yawVal = 0, pitchVal = 0, skewVal = 0, areaVal = 0;
  private boolean hasTarget = false;
  private boolean LED_Enable = false;

  private AprilTagFieldLayout aprilTagFieldLayout;

  public TargetVision() {
    this.m_camera = new PhotonCamera(VisionConstants.kCameraName);
    this.m_camera.setPipelineIndex(0);

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
    } catch (Exception e) {
    }

    m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        m_camera,
        VisionConstants.kCameraToRobot);
  }

  @Override
  public void periodic() {
    var result = this.m_camera.getLatestResult();
    if (result.hasTargets()) {
      this.yawVal = result.getBestTarget().getYaw();
      this.pitchVal = result.getBestTarget().getPitch();
      this.skewVal = result.getBestTarget().getSkew();
      this.areaVal = result.getBestTarget().getArea();
      this.hasTarget = true;

      SmartDashboard.putNumber("Yaw Value", yawVal);
      SmartDashboard.putNumber("Pitch Value", pitchVal);
      SmartDashboard.putNumber("Skew Value", skewVal);
      SmartDashboard.putNumber("Area Value", areaVal);
      SmartDashboard.putBoolean("LED On/Off", hasTarget);
    } else {
      this.hasTarget = false;

      if (LED_Enable) {
        cameraLEDOn();
        m_camera.setDriverMode(true);
      } else {
        cameraLEDOff();
        m_camera.setDriverMode(false);
      }
    }
  }

  public double getYawVal() {
    return this.yawVal;
  }

  public double getPitchVal() {
    return this.pitchVal;
  }

  public double getSkewVal() {
    return this.skewVal;
  }

  public double getAreaVal() {
    return this.areaVal;
  }

  public boolean hasTarget() {
    return this.hasTarget;
  }

  public void cameraLEDOn() {
    this.m_camera.setLED(VisionLEDMode.kOn);
  }

  public void cameraLEDOff() {
    this.m_camera.setLED(VisionLEDMode.kOff);
  }

  public void cameraLEDBlink() {
    this.m_camera.setLED(VisionLEDMode.kBlink);
  }

  public void cameraLEDToggle() {
    if (LED_Enable)
      LED_Enable = false;
    else
      LED_Enable = true;
  }

  public void cameraLED() {
    this.m_camera.setLED(VisionLEDMode.kDefault);
  }

  public double getRange() {
    double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kLensHeightMeters,
        VisionConstants.kTargetHeightMeters, VisionConstants.kLensPitchRadians, Units.degreesToRadians(getPitchVal()));
    SmartDashboard.putNumber("Camera Distance", range);
    return range;
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }
}
