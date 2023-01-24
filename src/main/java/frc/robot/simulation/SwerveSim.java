// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drivetrain;

/** Add your docs here. */
public class SwerveSim {
  private final Field2d m_field2d = new Field2d();
  private final Drivetrain m_drivetrain;

  public SwerveSim(Drivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
  }

  public void initSim() {
    // will eventually need sendablechooser for start locations
    Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
    m_field2d.setRobotPose(startPos);
    m_drivetrain.resetOdometry(m_field2d.getRobotPose(), startPos.getRotation());
    // m_drivetrain.resetOdometry();

    // may need this
    m_field2d.getObject("trajectory").setPose(new Pose2d());
  }

  // public void updateOdometry() {
  // m_poseEstimator.update(
  // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
  // m_rightEncoder.getDistance());

  // // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example
  // // -- on
  // // a real robot, this must be calculated based either on latency or
  // timestamps.
  // Optional<EstimatedRobotPose> result =
  // pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

  // if (result.isPresent()) {
  // EstimatedRobotPose camPose = result.get();
  // m_poseEstimator.addVisionMeasurement(
  // camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
  // m_fieldSim.getObject("Cam Est
  // Pos").setPose(camPose.estimatedPose.toPose2d());
  // } else {
  // // move it way off the screen to make it disappear
  // m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new
  // Rotation2d()));
  // }

  // m_fieldSim.getObject("Actual Pos").setPose(m_drivetrainSimulator.getPose());
  // m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
  // }

  public void simulationPeriodic() {
    m_field2d.setRobotPose(m_drivetrain.getPose());

    SmartDashboard.putData("field2d", m_field2d);
  }
}
