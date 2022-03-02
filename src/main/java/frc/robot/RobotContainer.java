// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoCommandBuilder;
import frc.robot.subsystems.*;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import io.github.oblarg.oblog.annotations.Log;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);

  private final Vision m_vision = new Vision();

  private final IntakeArm m_intakeArm = new IntakeArm();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Feeder m_feeder = new Feeder();
  public final Shooter m_shooter = new Shooter();
  private final Hood m_hood = new Hood();
  private final Turret m_turret = new Turret();
  private final Drivetrain m_drivetrain = new Drivetrain(m_turret::getRotation2d);

  @Log(name = "Auto Chooser")
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_vision.setTranslationConsumer(m_drivetrain::addVisionMeasurement);
    configureButtonBindings();

    m_autoChooser.setDefaultOption(
        "2 Ball Right Side",
        AutoCommandBuilder.get2BallAuto(
            m_drivetrain, m_vision, m_intake, m_intakeArm, m_indexer, m_feeder, m_shooter));
    m_autoChooser.addOption(
        "3 Ball Right Side",
        AutoCommandBuilder.get3BallAuto(
            m_drivetrain, m_vision, m_intake, m_intakeArm, m_indexer, m_feeder, m_shooter));
    m_autoChooser.addOption(
        "4 Ball Right Side",
        AutoCommandBuilder.get4BallAuto(
            m_drivetrain, m_vision, m_intake, m_intakeArm, m_indexer, m_feeder, m_shooter));
    m_autoChooser.addOption(
        "5 Ball Right Side",
        AutoCommandBuilder.get5BallAuto(
            m_drivetrain, m_vision, m_intake, m_intakeArm, m_indexer, m_feeder, m_shooter));
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
    this.configureCargoHandlingCommands();
  }

  private void configureDrivetrainCommands() {
    m_drivetrain.setDefaultCommand(
        new DriveWithController(
            m_driver::getLeftY,
            m_driver::getLeftX,
            m_driver::getRightX,
            () -> {
              return !m_driver.rightBumper().get();
            },
            m_drivetrain));

    m_driver
        .leftBumper()
        .whileHeld(
            new DriveFacingHub(
                m_driver::getLeftY,
                m_driver::getLeftX,
                () -> {
                  return !m_driver.rightBumper().get();
                },
                m_drivetrain));

    m_driver.x().whenPressed(new InstantCommand(m_drivetrain::zeroHeading));
  }

  private void configureVisionCommands() {
    // Cycle LED Mode when start button pressed
    m_driver.start().whenPressed(new InstantCommand(m_vision::cycleLEDMode));
  }

  private void configureCargoHandlingCommands() {
    // Default commands for intake, intake arm, shooter, and indexers are to turn them off
    m_shooter.setDefaultCommand(CargoHandlingCommandBuilder.getShooterOffCommand(m_shooter));
    m_intake.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeOffCommand(m_intake));
    m_intakeArm.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeArmOffCommand(m_intakeArm));
    m_indexer.setDefaultCommand(CargoHandlingCommandBuilder.getIndexerOffCommand(m_indexer));
    m_feeder.setDefaultCommand(CargoHandlingCommandBuilder.getFeederOffCommand(m_feeder));

    // Default commands for turret and hood are to auto-aim based on robot pose/distance
    m_turret.setDefaultCommand(
        CargoHandlingCommandBuilder.getTurretAutoAimCommand(
            m_turret, m_drivetrain::getPoseMeters, m_drivetrain::getVelocityMetersPerSecond));

    m_hood.setDefaultCommand(
        CargoHandlingCommandBuilder.getHoodAutoAimCommand(
            m_hood,
            m_drivetrain::getDistanceToHubMeters,
            m_drivetrain::getVelocityMetersPerSecond));

    // Intake on driver right trigger: put intake arm down, then run intake and low indexer (until
    // stopped by hitting high indexer path)
    m_driver
        .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
        .whileHeld(CargoHandlingCommandBuilder.getIntakeCommand(m_intake, m_intakeArm, m_indexer))
        .whenReleased(CargoHandlingCommandBuilder.getIntakeArmUpCommand(m_intakeArm));

    // Set shooter on operator left trigger: based on distance to hub
    m_operator
        .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
        .whileHeld(
            CargoHandlingCommandBuilder.getShooterAutoSetCommand(
                m_shooter,
                m_drivetrain::getDistanceToHubMeters,
                m_drivetrain::getVelocityMetersPerSecond));

    // Feed to shooter on operator right bumper: waits until shooter at setpoint
    m_operator
        .rightBumper()
        .whileHeld(
            CargoHandlingCommandBuilder.getIndexToShooterCommand(m_indexer, m_feeder, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected().andThen(AutoCommandBuilder.setLEDsAutoCommand(m_vision));
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    m_drivetrain.drive(0, 0, 0, false);
  }
}
