package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;

public class AlignToTarget extends CommandBase {
  private PIDController m_angleController = new PIDController(kPAngle, 0, kDAngle);
  private Drivetrain m_drivetrain;
  private PhotonCamera m_camera;

  public AlignToTarget(Drivetrain drivetrain, PhotonCamera camera) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_angleController.setTolerance(1);
    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    var result = m_camera.getLatestResult();

    double rotationSpeed;

    System.out.println(result.hasTargets());

    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = m_angleController.calculate(result.getBestTarget().getYaw(), 0);
      System.out.println("yaw: " + result.getBestTarget().getYaw());

    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
    System.out.println("rotation speed: " + rotationSpeed);

    m_drivetrain.drive(0, 0, rotationSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_angleController.atSetpoint();
  }
}
