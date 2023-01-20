package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.TargetVision;

public class FollowTag extends CommandBase {
  private Drivetrain m_drivetrain;
  private TargetVision m_targetVision;
  private WL_CommandXboxController m_controller;
  private boolean hasTarget;

  PIDController linearVisionController = new PIDController(VisionConstants.kVisionLinearP, 0, VisionConstants.kVisionLinearD);
  PIDController rotationVisionController = new PIDController(VisionConstants.kVisionAngularP, 0, VisionConstants.kVisionAngularD);

  public FollowTag(TargetVision vision, Drivetrain drivetrain, WL_CommandXboxController controller) {
      this.m_targetVision = vision;
      this.m_drivetrain = drivetrain;
      this.m_controller = controller;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (m_targetVision.hasTarget()) {
      this.hasTarget = true;

      double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kLensHeightMeters, VisionConstants.kTargetHeightMeters, VisionConstants.kLensPitchRadians, Units.degreesToRadians(m_targetVision.getPitchVal()));

      m_drivetrain.drive(new Translation2d(-linearVisionController.calculate(range, VisionConstants.kGoalRangeMeters), 0), -rotationVisionController.calculate(m_targetVision.getYawVal(), 0), false, true);
    } else {
      this.hasTarget = false;
      m_drivetrain.drive(new Translation2d(0, 0), 0, false, false);
    }
  }
}
