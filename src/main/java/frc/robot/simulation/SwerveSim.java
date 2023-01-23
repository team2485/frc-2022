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
  private Field2d m_field2d = new Field2d();
  public final Drivetrain m_drivetrain;

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
    // m_field2d.getObject("trajecotry").setPose(new Pose2d());
  }

  public void simulationPeriodic() {
    m_field2d.setRobotPose(m_drivetrain.getPose());

    SmartDashboard.putData("field2d", m_field2d);
  }
}
