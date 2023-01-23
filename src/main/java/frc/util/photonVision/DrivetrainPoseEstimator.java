package frc.util.photonVision;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class DrivetrainPoseEstimator {
  // sensors for pose estimation
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Swerve.pigeonID);
  private PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
  private Drivetrain drivetrain = new Drivetrain();

  // kalman filter config
  Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
  Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

  public final SwerveDrivePoseEstimator m_poseEstimator;

  public DrivetrainPoseEstimator() {
    m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, gyro.getRotation2d(),
        drivetrain.getModulePositions(), new Pose2d());
  }

  // perform periodic pose estimation tasks
  public void update() {
    m_poseEstimator.update(gyro.getRotation2d(), drivetrain.getModulePositions());

    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      var imageCaptureTime = result.getTimestampSeconds();
      var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
      var camPose = VisionConstants.kBlueMiddleTagAbsolutePos.transformBy(camToTargetTrans.inverse());
      m_poseEstimator.addVisionMeasurement(camPose.transformBy(VisionConstants.kCameraToRobot).toPose2d(),
          imageCaptureTime);
    }
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating
   * to the software
   * when you have manually moved your robot in a particular position on the field
   * (EX: when you
   * place it on the field at the start of the match).
   */
  public void resetToPose(Pose2d pose) {
    m_poseEstimator.resetPosition(gyro.getRotation2d(), drivetrain.getModulePositions(), pose);
  }

  /** @return The current best-guess at drivetrain position on the field. */
  public Pose2d getPoseEst() {
    return m_poseEstimator.getEstimatedPosition();
  }
}
