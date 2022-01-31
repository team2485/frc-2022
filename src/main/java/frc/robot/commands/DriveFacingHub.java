// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.team2485.WarlordsLib.oi.Deadband;
import java.util.function.DoubleSupplier;

public class DriveFacingHub extends CommandBase {
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;

  private final Drivetrain m_drivetrain;

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(kDriveSlewRate);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(kDriveSlewRate);

  private final PIDController m_anglePIDController = new PIDController(kPAngleHubTracking, 0, 0);

  public DriveFacingHub(
      DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, Drivetrain drivetrain) {

    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;

    this.m_drivetrain = drivetrain;

    Shuffleboard.getTab("Drivetrain").add(m_anglePIDController);

    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("xbox right x", m_xSpeedSupplier.getAsDouble());
    SmartDashboard.putNumber("xbox left x", m_ySpeedSupplier.getAsDouble());

    final double xSpeed =
        -m_xSpeedLimiter.calculate(
                Deadband.cubicScaledDeadband(m_xSpeedSupplier.getAsDouble(), kDriverLeftYDeadband))
            * kTeleopMaxSpeedMetersPerSecond;

    final double ySpeed =
        -m_ySpeedLimiter.calculate(
                Deadband.cubicScaledDeadband(m_ySpeedSupplier.getAsDouble(), kDriverLeftXDeadband))
            * kTeleopMaxSpeedMetersPerSecond;

    // Find hub position relative to the robot. The rotation of the hub relative to the robot is
    // used as the error for PID.
    Pose2d hubPositionRobotRelative = kHubCenterPosition.relativeTo(m_drivetrain.getPoseMeters());
    double drivetrainRotationRadians = m_drivetrain.getPoseMeters().getRotation().getRadians();
    double angleError = hubPositionRobotRelative.getRotation().getRadians();
    double desiredAngularVelocity =
        m_anglePIDController.calculate(
            drivetrainRotationRadians, drivetrainRotationRadians + angleError);

    m_drivetrain.drive(xSpeed, ySpeed, desiredAngularVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
