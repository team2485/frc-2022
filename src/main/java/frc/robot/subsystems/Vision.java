package frc.robot.subsystems;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2485.WarlordsLib.math.CircleFitter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Much of this class is based on FRC 6328's vision code:
 * https://github.com/Mechanical-Advantage/RobotCode2022/blob/f4fec2247e47195467eccd504fe44c674e45786a/src/main/java/frc/robot/subsystems/vision/Vision.java
 */
public class Vision extends SubsystemBase implements Loggable {
  private PhotonCamera m_camera = new PhotonCamera(kCameraName);

  public enum LEDSetMode {
    kAlwaysOn,
    kAlwaysOff,
    kAuto
  }

  @Log private boolean m_LEDsOn = false;

  private LEDSetMode m_LEDSetMode = LEDSetMode.kAuto;
  private boolean m_forceLEDs = false;

  private Consumer<TimestampedTranslation2d> m_translationConsumer;

  private double m_lastCaptureTimestamp = 0.0;
  private double m_imageCaptureTimestamp;
  private double[] m_cornerX = new double[] {};
  private double[] m_cornerY = new double[] {};

  private Timer m_targetGraceTimer = new Timer(); // times amount of time since target loss
  DoubleArrayLogEntry m_target1TopLeftCornerLog =
      new DoubleArrayLogEntry(DataLogManager.getLog(), "/vision/target1/topLeft");
  DoubleArrayLogEntry m_target1TopRightCornerLog =
      new DoubleArrayLogEntry(DataLogManager.getLog(), "/vision/target1/topRight");
  DoubleArrayLogEntry m_target1BottomLeftCornerLog =
      new DoubleArrayLogEntry(DataLogManager.getLog(), "/vision/target1/bottomLeft");
  DoubleArrayLogEntry m_target1BottomRightCornerLog =
      new DoubleArrayLogEntry(DataLogManager.getLog(), "/vision/target1/bottomRight");

  public Vision() {
    m_targetGraceTimer.start();

    NetworkTableInstance.getDefault()
        .getEntry("/photonvision/" + kCameraName + "/latencyMillis")
        .addListener(
            event -> {
              PhotonPipelineResult result = m_camera.getLatestResult();
              double timestamp = Timer.getFPGATimestamp() - (result.getLatencyMillis() / 1000.0);

              List<Double> cornerXList = new ArrayList<>();
              List<Double> cornerYList = new ArrayList<>();
              for (PhotonTrackedTarget target : result.getTargets()) {
                for (TargetCorner corner : target.getCorners()) {
                  cornerXList.add(corner.x);
                  cornerYList.add(corner.y);
                }
              }

              synchronized (this) {
                m_imageCaptureTimestamp = timestamp;
                m_cornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                m_cornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
              }
            },
            EntryListenerFlags.kUpdate);
  }

  public void setLEDMode(LEDSetMode mode) {
    this.m_LEDSetMode = mode;
  }

  @Log.ToString(name = "LED Set Mode", tabName = "RobotContainer")
  public LEDSetMode getLEDSetMode() {
    return m_LEDSetMode;
  }

  /**
   * If current LED mode is Auto, turn to AlwaysOff. If current LED mode is AlwaysOff, turn to
   * AlwaysOn. If current LED mode is AlwaysOn, turn to Auto.
   */
  public void cycleLEDMode() {
    System.out.println("Led mode: " + m_LEDSetMode);
    switch (m_LEDSetMode) {
      case kAuto:
        this.setLEDMode(LEDSetMode.kAlwaysOff);
        break;
      case kAlwaysOff:
        this.setLEDMode(LEDSetMode.kAlwaysOn);
        break;
      case kAlwaysOn:
        this.setLEDMode(LEDSetMode.kAuto);
        break;
      default:
        this.setLEDMode(LEDSetMode.kAuto);
        break;
    }
  }

  /** Use to enable LEDs continuously while override is "Auto" */
  public void setForceLeds(boolean on) {
    m_forceLEDs = on;
  }

  private void setLEDs(boolean enabled) {
    m_camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setTranslationConsumer(Consumer<TimestampedTranslation2d> consumer) {
    m_translationConsumer = consumer;
  }

  @Override
  public void periodic() {
    // count targets if LEDs on
    int targetCount = m_LEDsOn ? m_cornerX.length / 4 : 0;
    // System.out.println("Target Count: " + targetCount);

    // Update LED idle state
    // Reset grace timer if targets found
    if (targetCount > 0) {
      m_targetGraceTimer.reset();
    }

    // Blink if no target found for more than grace period
    boolean idleOn =
        m_targetGraceTimer.get() < kTargetGraceSecs
            || Timer.getFPGATimestamp() % kBlinkPeriodSecs < kBlinkLengthSecs;

    // Update LED mode based on supplier/idle state/DS state
    switch (m_LEDSetMode) {
      case kAlwaysOn:
        m_LEDsOn = true;
        break;
      case kAlwaysOff:
        m_LEDsOn = false;
        break;
      case kAuto:
        if (m_forceLEDs) {
          m_LEDsOn = true;
        } else if (DriverStation.isDisabled()) {
          m_LEDsOn = false;
        } else if (DriverStation.isAutonomous()) {
          m_LEDsOn = true;
        } else {
          m_LEDsOn = idleOn;
        }
        break;
      default:
        m_LEDsOn = false;
        break;
    }

    this.setLEDs(m_LEDsOn);

    // Don't update translation if no new image
    if (m_imageCaptureTimestamp == m_lastCaptureTimestamp) {
      return;
    }

    m_lastCaptureTimestamp = m_imageCaptureTimestamp;

    // If we have enough targets, calculate camera to target translation
    if (targetCount >= kMinTargetCount) {
      List<Translation2d> cameraToTargetTranslations = new ArrayList<>();

      for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
        List<Translation2d> corners = new ArrayList<>();
        double totalX = 0.0, totalY = 0.0; // for averaging

        for (int i = targetIndex * 4; i < targetIndex * 4 + 4; i++) {
          if (i < m_cornerX.length && i < m_cornerY.length) {
            // add x and y of corner to our translation matrix
            corners.add(new Translation2d(m_cornerX[i], m_cornerY[i]));
            totalX += m_cornerX[i];
            totalY += m_cornerY[i];
          }
        }

        Translation2d targetAvg = new Translation2d(totalX / 4, totalY / 4);
        corners = sortCorners(corners, targetAvg);

        for (int i = 0; i < corners.size(); i++) {
          Translation2d cornerTranslation =
              solveCameraToTargetTranslation(
                  corners.get(i), i < 2 ? kVisionTargetHeightUpper : kVisionTargetHeightLower);
          if (cornerTranslation != null) {
            cameraToTargetTranslations.add(cornerTranslation);
          }
        }
        if (targetIndex == 0) {
          m_target1TopLeftCornerLog.append(
              new double[] {corners.get(0).getX(), corners.get(0).getY()});
          m_target1TopRightCornerLog.append(
              new double[] {corners.get(1).getX(), corners.get(1).getY()});
          m_target1BottomLeftCornerLog.append(
              new double[] {corners.get(2).getX(), corners.get(2).getY()});
          m_target1BottomRightCornerLog.append(
              new double[] {corners.get(3).getX(), corners.get(3).getY()});
        }
      }

      if (cameraToTargetTranslations.size() >= kMinTargetCount * 4) {
        Translation2d cameraToTargetCenterTranslation =
            CircleFitter.fit(kVisionTargetRadius, cameraToTargetTranslations, kCircleFitPrecision);

        m_translationConsumer.accept(
            new TimestampedTranslation2d(
                m_imageCaptureTimestamp - kExtraLatencySecs, cameraToTargetCenterTranslation));
      }
    }
  }

  /**
   * Sorts list of corners to be in order: top left, top right, bottom left, bottom right
   *
   * @return
   */
  private static List<Translation2d> sortCorners(
      List<Translation2d> corners, Translation2d average) {
    // Find top corners
    Integer topLeftIndex = null;
    Integer topRightIndex = null;
    double minPosRads = Math.PI;
    double minNegRads = Math.PI;
    for (int i = 0; i < corners.size(); i++) {
      Translation2d corner = corners.get(i);
      double angleRad =
          new Rotation2d(corner.getX() - average.getX(), average.getY() - corner.getY())
              .minus(Rotation2d.fromDegrees(90))
              .getRadians();
      if (angleRad > 0) {
        if (angleRad < minPosRads) {
          minPosRads = angleRad;
          topLeftIndex = i;
        }
      } else {
        if (Math.abs(angleRad) < minNegRads) {
          minNegRads = Math.abs(angleRad);
          topRightIndex = i;
        }
      }
    }

    Integer lowerIndex1 = null, lowerIndex2 = null;
    // Find lower corners (don't need left or right)
    for (int i = 0; i < corners.size(); i++) {
      boolean alreadySaved = false;
      if (topLeftIndex != null) {
        if (topLeftIndex.equals(i)) {
          alreadySaved = true;
        }
      }
      if (topRightIndex != null) {
        if (topRightIndex.equals(i)) {
          alreadySaved = true;
        }
      }
      if (!alreadySaved) {
        if (lowerIndex1 == null) {
          lowerIndex1 = i;
        } else {
          lowerIndex2 = i;
        }
      }
    }

    // Produce final list
    List<Translation2d> sortedCorners = new ArrayList();
    if (topLeftIndex != null) {
      sortedCorners.add(corners.get(topLeftIndex));
    }

    if (topRightIndex != null) {
      sortedCorners.add(corners.get(topRightIndex));
    }

    if (lowerIndex1 != null) {
      sortedCorners.add(corners.get(lowerIndex1));
    }

    if (lowerIndex2 != null) {
      sortedCorners.add(corners.get(lowerIndex2));
    }

    return sortedCorners;
  }

  private Translation2d solveCameraToTargetTranslation(Translation2d corner, double targetHeight) {
    double xPixels = corner.getX();
    double yPixels = corner.getY();

    // Robot frame of reference. Scaled -1 to 1.
    double nY = -(xPixels - (double) kCameraPixelsX / 2) / ((double) kCameraPixelsX / 2);
    double nZ = -(yPixels - (double) kCameraPixelsX / 2) / ((double) kCameraPixelsX / 2);

    Translation2d xzPlaneTranslation =
        new Translation2d(1, kCameraViewportRatioH / 2.0 * nZ).rotateBy(kCameraPitch);

    double x = xzPlaneTranslation.getX();
    double y = kCameraViewportRatioW / 2.0 * nY; // Y viewport from horizontal
    double z = xzPlaneTranslation.getY();

    double heightLensToTarget = kLensHeightMeters - targetHeight;
    if ((z < 0.0) == (heightLensToTarget > 0.0)) {
      double scaling = heightLensToTarget / -z;
      double distanceLensToTarget = Math.hypot(x, y) * scaling;
      Rotation2d targetPitchFromLens = new Rotation2d(x, y);

      return new Translation2d(
          distanceLensToTarget * targetPitchFromLens.getCos(),
          distanceLensToTarget * targetPitchFromLens.getSin());
    }

    return null;
  }

  public static class TimestampedTranslation2d {
    public final double timestamp;
    public final Translation2d translation;

    public TimestampedTranslation2d(double timestamp, Translation2d translation) {
      this.timestamp = timestamp;
      this.translation = translation;
    }
  }
}
