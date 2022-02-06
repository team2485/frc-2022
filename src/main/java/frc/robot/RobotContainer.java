// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FlywheelConstants.*;
import static frc.robot.Constants.OIConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Flywheel;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);
  Flywheel m_flywheel = new Flywheel();

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Vision m_vision = new Vision();

  @Log(name = "Field Relative")
  private boolean m_fieldRelativeToggle = true;

  @Log(name = "Facing Hub")
  private boolean m_hubFacingToggle = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_vision.setTranslationConsumer(m_drivetrain::addVisionMeasurement);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
    this.configureVisionCommands();
  }

  private void configureDrivetrainCommands() {
    m_drivetrain.setDefaultCommand(
        new DriveWithController(
            m_driver::getLeftY,
            m_driver::getLeftX,
            m_driver::getRightX,
            () -> {
              return !m_driver.getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold).get();
            },
            m_drivetrain));

    m_driver
        .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
        .whileHeld(
            new DriveFacingHub(
                m_driver::getLeftY,
                m_driver::getLeftX,
                () -> {
                  return !m_driver
                      .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
                      .get();
                },
                m_drivetrain));

    m_driver.x().whenPressed(new InstantCommand(m_drivetrain::zeroHeading));
  }

  private void configureVisionCommands() {
    // Cycle LED Mode when start button pressed
    m_driver.start().whenPressed(new InstantCommand(m_vision::cycleLEDMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // load path from deploy/pathplanner folder
    PathPlannerTrajectory testPath =
        PathPlanner.loadPath(
            "Blue 4 Ball Bottom Side (NOH)",
            kAutoMaxSpeedMetersPerSecond,
            kAutoMaxAccelerationMetersPerSecondSquared);

    // put trajectory on Glass's Field2d widget
    m_drivetrain.getField2d().getObject("traj").setTrajectory(testPath);

    // create controller for robot angle
    var thetaController =
        new ProfiledPIDController(kPAutoThetaController, 0, 0, kAutoThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // create reset odometry command (at start of path)
    InstantCommand resetOdometry =
        new InstantCommand(
            () -> {
              m_drivetrain.resetOdometry(testPath.getInitialPose(), false);
            });

    // create command to follow path
    PPSwerveControllerCommand testPathCommand =
        new PPSwerveControllerCommand(
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

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    m_drivetrain.drive(0, 0, 0, false);
  }
}
