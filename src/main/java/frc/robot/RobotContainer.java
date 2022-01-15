// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);

  private final Drivetrain m_drivetrain = new Drivetrain();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
  }

  private void configureDrivetrainCommands() {
    m_drivetrain.setDefaultCommand(new DriveWithController(m_driver, m_drivetrain));

    m_driver.x().whenPressed(new InstantCommand(m_drivetrain::zeroHeading));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                kAutoMaxSpeedMetersPerSecond,
                kAutoMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics);

    PathPlannerTrajectory testPath = 
        PathPlanner.loadPath("Test Path", 
                              kAutoMaxSpeedMetersPerSecond, 
                              kAutoMaxAccelerationMetersPerSecondSquared);

    m_drivetrain.getField2d().getObject("traj").setTrajectory(testPath);
                              
    var thetaController =
        new ProfiledPIDController(
              kPAutoThetaController, 0, 0, kAutoThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    InstantCommand resetOdometry = new InstantCommand(
            ()->{m_drivetrain.resetOdometry(testPath.getInitialPose());}
    );

    HolonomicSwerveControllerCommand testPathCommand = 
      new HolonomicSwerveControllerCommand(
        testPath, 
        m_drivetrain::getPoseMeters,
        kDriveKinematics,
        new PIDController(kPAutoXController, 0, 0),
        new PIDController(kPAutoYController, 0, 0),
        thetaController, 
        m_drivetrain::setModuleStates,
        m_drivetrain);
            
    return resetOdometry.andThen(testPathCommand);
  }

  public void disabledInit() {
    m_drivetrain.drive(0, 0, 0, false);
  }

  public void disabledPeriodic() {
    //m_drivetrain.drive(0, 0, 0, false);
  }
}