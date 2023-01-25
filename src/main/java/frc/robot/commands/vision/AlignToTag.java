package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.util.photonVision.PhotonCamera;
import frc.util.photonVision.PhotonTrackedTarget;

public class AlignToTag extends CommandBase {
  private final PhotonCamera m_camera;
  private final Drivetrain m_drivetrain;
  private final Supplier<Pose2d> m_poseProvider;

  private final ProfiledPIDController m_XController = new ProfiledPIDController(4, 0, 0, VisionConstants.kXConstraints);
  private final ProfiledPIDController m_YController = new ProfiledPIDController(4, 0, 0, VisionConstants.kYConstraints);
  private final ProfiledPIDController m_OmegaController = new ProfiledPIDController(2, 0, 0,
      VisionConstants.kOmegaConstraints);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

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
}
