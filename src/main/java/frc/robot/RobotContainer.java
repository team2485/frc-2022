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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.cargoHandling.indexing.*;
import frc.robot.subsystems.drive.*;
import frc.team2485.WarlordsLib.oi.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);

  // private final Vision m_vision = new Vision();

  // private final IntakeArm m_intakeArm = new IntakeArm();
  // private final Intake m_intake = new Intake();
  private final LowIndexer m_lowIndexer = new LowIndexer();
  private final HighIndexer m_highIndexer = new HighIndexer();
  // public final Shooter m_shooter = new Shooter();
  // // private final Hood m_hood = new Hood();
  // private final Turret m_turret = new Turret();
  // private final Drivetrain m_drivetrain = new Drivetrain(m_turret::getRotation2d);

  // @Log(name = "Auto Chooser")
  // private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_vision.setTranslationConsumer(m_drivetrain::addVisionMeasurement);
    configureButtonBindings();

    // m_autoChooser.setDefaultOption(
    //     "2 Ball Right Side",
    //     AutoCommandBuilder.get2BallAuto(
    //         m_drivetrain, m_vision, m_intake, m_intakeArm, m_lowIndexer, m_highIndexer,
    // m_shooter));
    // m_autoChooser.addOption(
    //     "3 Ball Right Side",
    //     AutoCommandBuilder.get3BallAuto(
    //         m_drivetrain, m_vision, m_intake, m_intakeArm, m_lowIndexer, m_highIndexer,
    // m_shooter));
    // m_autoChooser.addOption(
    //     "4 Ball Right Side",
    //     AutoCommandBuilder.get4BallAuto(
    //         m_drivetrain, m_vision, m_intake, m_intakeArm, m_lowIndexer, m_highIndexer,
    // m_shooter));
    // m_autoChooser.addOption(
    //     "5 Ball Right Side",
    //     AutoCommandBuilder.get5BallAuto(
    //         m_drivetrain, m_vision, m_intake, m_intakeArm, m_lowIndexer, m_highIndexer,
    // m_shooter));
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // this.configureDrivetrainCommands();
    // this.configureVisionCommands();
    // this.configureCargoHandlingCommands();
  }

  private void configureDrivetrainCommands() {
    // m_drivetrain.setDefaultCommand(
    //     new DriveWithController(
    //         m_driver::getLeftY,
    //         m_driver::getLeftX,
    //         m_driver::getRightX,
    //         () -> {
    //           return !m_driver.rightBumper().get();
    //         },
    //         m_drivetrain));

    // m_driver
    //     .leftBumper()
    //     .whileHeld(
    //         new DriveFacingHub(
    //             m_driver::getLeftY,
    //             m_driver::getLeftX,
    //             () -> {
    //               return !m_driver.rightBumper().get();
    //             },
    //             m_drivetrain));

    // m_driver.x().whenPressed(new InstantCommand(m_drivetrain::zeroHeading));
  }

  private void configureVisionCommands() {
    // Cycle LED Mode when start button pressed
    // m_driver.start().whenPressed(new InstantCommand(m_vision::cycleLEDMode));
  }

  private void configureCargoHandlingCommands() {
    // Default commands for intake, intake arm, shooter, and indexers are to turn them off
    // m_shooter.setDefaultCommand(CargoHandlingCommandBuilder.getShooterOffCommand(m_shooter));
    // m_intake.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeOffCommand(m_intake));
    // m_intakeArm.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeArmOffCommand(m_intakeArm));
    // m_lowIndexer.setDefaultCommand(
    //     CargoHandlingCommandBuilder.getLowIndexerOffCommand(m_lowIndexer));
    // m_highIndexer.setDefaultCommand(
    //     CargoHandlingCommandBuilder.getHighIndexerOffCommand(m_highIndexer));

    // Default commands for turret and hood are to auto-aim based on robot pose/distance
    // m_turret.setDefaultCommand(
    //     CargoHandlingCommandBuilder.getTurretAutoAimCommand(m_turret,
    // m_drivetrain::getPoseMeters));

    // m_hood.setDefaultCommand(
    //     CargoHandlingCommandBuilder.getHoodAutoAimCommand(
    //         m_hood, m_drivetrain::getDistanceToHubMeters));

    // // Intake on driver right trigger: put intake arm down, then run intake and low indexer
    // (until
    // // stopped by hitting high indexer path)
    // m_driver
    //     .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
    //     .whileHeld(
    //         CargoHandlingCommandBuilder.getIntakeCommand(m_intake, m_intakeArm, m_lowIndexer))
    //     .whenReleased(CargoHandlingCommandBuilder.getIntakeArmUpCommand(m_intakeArm));

    // // Set shooter on operator left trigger: based on distance to hub
    // m_operator
    //     .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
    //     .whileHeld(
    //         CargoHandlingCommandBuilder.getShooterAutoSetCommand(
    //             m_shooter, m_drivetrain::getDistanceToHubMeters));

    // // Feed to shooter on operator right bumper: waits until shooter at setpoint
    // m_operator
    //     .rightBumper()
    //     .whileHeld(
    //         CargoHandlingCommandBuilder.getIndexToShooterCommand(
    //             m_lowIndexer, m_highIndexer, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_autoChooser.getSelected().andThen(AutoCommandBuilder.setLEDsAutoCommand(m_vision));
    return null;
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    // m_drivetrain.drive(0, 0, 0, false);
  }
}
