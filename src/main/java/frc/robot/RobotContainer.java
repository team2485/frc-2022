// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.cargoHandling.*;
import frc.team2485.WarlordsLib.oi.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);

  private final Feeder m_feeder = new Feeder();
  public final Shooter m_shooter = new Shooter();
  private final Hood m_hood = new Hood();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.configureCargoHandlingCommands();
  }

  private void configureCargoHandlingCommands() {
    // Default commands for intake, intake arm, shooter, and indexers are to turn them off
    m_feeder.setDefaultCommand(new RunCommand(() -> m_feeder.setPercentOutput(0), m_feeder));

    m_driver
        .b()
        .whenHeld(
            new InstantCommand(() -> m_feeder.engageBelts(true), m_feeder)
                .andThen(
                    new ConditionalCommand(
                        new RunCommand(() -> m_feeder.setPercentOutput(0.5), m_feeder),
                        new InstantCommand(() -> m_feeder.setPercentOutput(0), m_feeder),
                        m_shooter::atSetpoint)));

    m_driver.a().whenPressed(new InstantCommand(() -> m_feeder.engageBelts(false), m_feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {}
}
