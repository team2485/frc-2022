package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class AlignToTag extends CommandBase {
  private final PhotonCamera m_camera;
  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;

  private final ProfiledPIDController m_XController = new ProfiledPIDController(1, 0, 0, VisionConstants.kXConstraints);
  private final ProfiledPIDController m_YController = new ProfiledPIDController(1, 0, 0, VisionConstants.kYConstraints);
  private final ProfiledPIDController m_OmegaController = new ProfiledPIDController(.5, 0, 0,
      VisionConstants.kOmegaConstraints);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  public AlignToTag(PhotonCamera camera, Drivetrain drivetrain, Supplier<Pose2d> poseProvider) {
    this.m_camera = camera;
    this.m_drivetrain = drivetrain;
    this.m_poseProvider = poseProvider;

    m_XController.setTolerance(0.1);
    m_YController.setTolerance(0.1);
    m_OmegaController.setTolerance(Units.degreesToRadians(3));
    m_OmegaController.enableContinuousInput(-1, 1);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    var robotPose = m_poseProvider.get();
    m_OmegaController.reset(robotPose.getRotation().getRadians());
    m_XController.reset(robotPose.getX());
    m_YController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = m_poseProvider.get();
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      // find tag to chase
      var targetOpt = result.getTargets().stream().filter(t -> t.getFiducialId() == VisionConstants.kTagOfInterest)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          lastTarget = target;

          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
              camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

          var cameraPose = robotPose.transformBy(
              new Transform2d(VisionConstants.kCameraToRobot.getTranslation().toTranslation2d(), new Rotation2d())
                  .inverse());
          Pose2d targetPose = cameraPose.transformBy(transform);

          goalPose = targetPose.transformBy(VisionConstants.kTagToGoal);
        }

        if (null != goalPose) {
          m_XController.setGoal(goalPose.getX());
          m_YController.setGoal(goalPose.getY());
          m_OmegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }

    var xSpeed = m_XController.calculate(robotPose.getX());
    if (m_XController.atGoal()) {
      xSpeed = 0;
    }
    SmartDashboard.putNumber("x speed", xSpeed);

    var ySpeed = m_YController.calculate(robotPose.getY());
    if (m_YController.atGoal()) {
      ySpeed = 0;
    }
    SmartDashboard.putNumber("y speed", ySpeed);

    var omegaSpeed = m_OmegaController.calculate(robotPose.getRotation().getRadians());
    if (m_OmegaController.atGoal()) {
      omegaSpeed = 0;
    }
    SmartDashboard.putNumber("omega speed", omegaSpeed);

    m_drivetrain.drive(new Translation2d(xSpeed, ySpeed), omegaSpeed, true, false);
  }
}
