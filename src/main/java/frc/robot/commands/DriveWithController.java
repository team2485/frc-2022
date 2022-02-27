// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;
import frc.team2485.WarlordsLib.oi.Deadband;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithController extends CommandBase {
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotSpeedSupplier;
  private final BooleanSupplier m_fieldRelative;

  private final Drivetrain m_drivetrain;

  public DriveWithController(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSpeedSupplier,
      BooleanSupplier fieldRelative,
      Drivetrain drivetrain) {

    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;
    this.m_rotSpeedSupplier = rotSpeedSupplier;
    this.m_fieldRelative = fieldRelative;

    this.m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("xbox right x", m_xSpeedSupplier.getAsDouble());
    SmartDashboard.putNumber("xbox left x", m_ySpeedSupplier.getAsDouble());
    SmartDashboard.putNumber("xbox left y", m_rotSpeedSupplier.getAsDouble());

    final double xSpeed =
        Deadband.cubicScaledDeadband(m_xSpeedSupplier.getAsDouble(), kDriverLeftYDeadband)
            * kTeleopMaxSpeedMetersPerSecond;

    final double ySpeed =
        Deadband.cubicScaledDeadband(m_ySpeedSupplier.getAsDouble(), kDriverLeftXDeadband)
            * kTeleopMaxSpeedMetersPerSecond;

    final double rot =
        Deadband.cubicScaledDeadband(m_rotSpeedSupplier.getAsDouble(), kDriverRightXDeadband)
            * kTeleopMaxAngularSpeedRadiansPerSecond;

    final boolean fieldRelative = m_fieldRelative.getAsBoolean();
    m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);

    // System.out.println(m_driver.getRightTriggerAxis());
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
