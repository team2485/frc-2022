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

    final int xSign = (int)(Math.abs(m_xSpeedSupplier.getAsDouble())/m_xSpeedSupplier.getAsDouble());
    final double xSpeed =
        map(-Deadband.cubicScaledDeadband(Math.abs(m_xSpeedSupplier.getAsDouble()), kDriverLeftYDeadband)
            * kTeleopMaxSpeedMetersPerSecond, kDriverLeftYDeadband, 1, 0, 1) * xSign;

    final int ySign = (int)(Math.abs(m_ySpeedSupplier.getAsDouble())/m_ySpeedSupplier.getAsDouble());
    final double ySpeed =
        map(-Deadband.cubicScaledDeadband(Math.abs(m_ySpeedSupplier.getAsDouble()), kDriverLeftXDeadband)
          * kTeleopMaxSpeedMetersPerSecond, kDriverLeftXDeadband, 1, 0, 1) * ySign;

    final int rotSign = (int)(Math.abs(m_rotSpeedSupplier.getAsDouble())/m_rotSpeedSupplier.getAsDouble());
    final double rot =
        map(-Deadband.cubicScaledDeadband(m_rotSpeedSupplier.getAsDouble(), kDriverRightXDeadband)
            * kTeleopMaxAngularSpeedRadiansPerSecond, kDriverRightXDeadband, 1, 0, 1) * rotSign;

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
