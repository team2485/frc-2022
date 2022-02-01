// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
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

  private NetworkTableEntry kP = Shuffleboard.getTab("Drivetrain").add("kP", 0).getEntry();
  private NetworkTableEntry kD = Shuffleboard.getTab("Drivetrain").add("kD", 0).getEntry();

  private final PIDController m_anglePIDController =
      new PIDController(kP.getDouble(0.2), 0, kD.getDouble(0.1));

  // private final ProfiledPIDController m_anglePIDController =
  //   new ProfiledPIDController(kPAutoThetaController, 0, 0, kAutoThetaControllerConstraints);

  public DriveFacingHub(
      DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, Drivetrain drivetrain) {

    m_anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_anglePIDController.setTolerance(0.2);

    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;

    this.m_drivetrain = drivetrain;

    // Shuffleboard.getTab("Drivetrain").add(m_anglePIDController);

    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_anglePIDController.setP(kP.getDouble(0.2));
    m_anglePIDController.setD(kD.getDouble(0.1));

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
    m_drivetrain.m_field.getObject("hub non-robot relative").setPose(kHubCenterPosition);
    m_drivetrain.m_field.getObject("hub robot relative").setPose(hubPositionRobotRelative);

    Translation2d hubMinusRobot =
        kHubCenterTranslation.minus(m_drivetrain.getPoseMeters().getTranslation());

    double desiredRotation = Math.atan(hubMinusRobot.getY() / hubMinusRobot.getX());

    SmartDashboard.putNumber("alpha", desiredRotation);

    m_drivetrain.driveWithRotationPosition(xSpeed, ySpeed, desiredRotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_anglePIDController.atSetpoint();
  }
}
