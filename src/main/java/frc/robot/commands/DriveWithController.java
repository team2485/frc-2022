// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import frc.robot.subsystems.Drivetrain;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import frc.team2485.WarlordsLib.oi.Deadband;

public class DriveWithController extends CommandBase {
    private final CommandXboxController m_driver;
    private final Drivetrain m_drivetrain;

    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(kDriveSlewRate);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(kDriveSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(kDriveSlewRate);

    public DriveWithController(CommandXboxController driver, Drivetrain drivetrain) {
        this.m_driver = driver;
        this.m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("xbox right x", m_driver.getRightX());
        SmartDashboard.putNumber("xbox left x", m_driver.getLeftX());
        SmartDashboard.putNumber("xbox left y", m_driver.getLeftY());


        final double xSpeed = 
            -m_xSpeedLimiter.calculate(Deadband.squareScaleDeadband(m_driver.getLeftY(), kDriverLeftYDeadband)) 
            * kTeleopMaxSpeedMetersPerSecond;
        
        final double ySpeed = 
            -m_ySpeedLimiter.calculate(Deadband.squareScaleDeadband(m_driver.getLeftX(), kDriverLeftXDeadband))
            * kTeleopMaxSpeedMetersPerSecond;
        
        final double rot = 
            -m_rotLimiter.calculate(Deadband.squareScaleDeadband(m_driver.getRightX(), kDriverRightXDeadband))
            * kTeleopMaxAngularSpeedRadiansPerSecond;
        
        final boolean fieldRelative = !m_driver.y().get();
        m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
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
