// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Flywheel;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.FlywheelConstants.*;

public class RobotContainer {
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);
  Flywheel m_flywheel = new Flywheel();

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
    m_flywheel.setVelocityRotationsPerSecond(m_operator.getLeftY() * kFlywheelMaxSpeedRotationsPerSecond);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
