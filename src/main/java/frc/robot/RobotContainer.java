// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.ClimbStateMachine.ClimbState;
import frc.robot.subsystems.drive.*;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import io.github.oblarg.oblog.annotations.*;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);

  //   private final Vision m_vision = new Vision();

  private final IntakeArm m_intakeArm = new IntakeArm();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Feeder m_feeder = new Feeder();
  public final Shooter m_shooter = new Shooter();
  private final Hood m_hood = new Hood();
  private final Turret m_turret = new Turret();
  private final BallCounter m_ballCounter = new BallCounter(m_shooter::hasDipped);

  private final Drivetrain m_drivetrain =
      new Drivetrain(
          () -> {
            return new Rotation2d();
          });

  public final ClimbElevator m_climbElevator = new ClimbElevator();
  public final ClimbArm m_climbArm = new ClimbArm();
  public final ClimbStateMachine m_climbStateMachine = new ClimbStateMachine();

  @Log(name = "Auto Chooser")
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_vision.setTranslationConsumer(m_drivetrain::addVisionMeasurement);
    configureButtonBindings();

    // m_autoChooser.setDefaultOption(
    //     "2 Ball Right Side",
    //     AutoCommandBuilder.get2BallAuto(
    //         m_drivetrain,
    //         m_vision,
    //         m_intake,
    //         m_intakeArm,
    //         m_indexer,
    //         m_feeder,
    //         m_shooter,
    //         m_ballCounter));
    // m_autoChooser.addOption(
    //     "3 Ball Right Side",
    //     AutoCommandBuilder.get3BallAuto(
    //         m_drivetrain,
    //         m_vision,
    //         m_intake,
    //         m_intakeArm,
    //         m_indexer,
    //         m_feeder,
    //         m_shooter,
    //         m_ballCounter));
    // m_autoChooser.addOption(
    //     "4 Ball Right Side",
    //     AutoCommandBuilder.get4BallAuto(
    //         m_drivetrain,
    //         m_vision,
    //         m_intake,
    //         m_intakeArm,
    //         m_indexer,
    //         m_feeder,
    //         m_shooter,
    //         m_ballCounter));
    // m_autoChooser.addOption(
    //     "5 Ball Right Side",
    //     AutoCommandBuilder.get5BallAuto(
    //         m_drivetrain,
    //         m_vision,
    //         m_intake,
    //         m_intakeArm,
    //         m_indexer,
    //         m_feeder,
    //         m_shooter,
    //         m_ballCounter));
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
    // this.configureCargoHandlingCommands();
    // this.configureClimbCommands();
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
    // m_driver.start().whenPressed(new InstantCommand(m_vision::cycleLEDMode));
  }

  //   private void configureCargoHandlingCommands() {
  //     // Default commands for intake, intake arm, shooter, and indexers are to turn them off
  //     m_shooter.setDefaultCommand(CargoHandlingCommandBuilder.getShooterOffCommand(m_shooter));
  //     m_intake.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeOffCommand(m_intake));
  //
  // m_intakeArm.setDefaultCommand(CargoHandlingCommandBuilder.getIntakeArmOffCommand(m_intakeArm));
  //     m_indexer.setDefaultCommand(CargoHandlingCommandBuilder.getIndexerOffCommand(m_indexer));
  //     m_feeder.setDefaultCommand(CargoHandlingCommandBuilder.getFeederOffCommand(m_feeder));

  //     // // Default commands for turret and hood are to auto-aim based on robot pose/distance
  //     // m_turret.setDefaultCommand(
  //     //     CargoHandlingCommandBuilder.getTurretAutoAimCommand(
  //     //         m_turret, m_drivetrain::getPoseMeters,
  // m_drivetrain::getVelocityMetersPerSecond));

  //     // m_hood.setDefaultCommand(
  //     //     CargoHandlingCommandBuilder.getHoodAutoAimCommand(
  //     //         m_hood,
  //     //         m_drivetrain::getDistanceToHubMeters,
  //     //         m_drivetrain::getFieldRelativeVelocityMetersPerSecond));

  //     // Intake on driver right trigger: put intake arm down, then run intake and low indexer
  // (until
  //     // stopped by hitting high indexer path)
  //     // m_driver
  //     //     .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
  //     //     .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
  //     //     .whileActiveContinuous(
  //     //         CargoHandlingCommandBuilder.getIntakeCommand(
  //     //             m_intake, m_intakeArm, m_indexer, m_ballCounter))
  //     //     .whenInactive(CargoHandlingCommandBuilder.getIntakeArmUpCommand(m_intakeArm));

  //     // Set shooter on operator left trigger: based on distance to hub
  //     m_operator
  //         .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
  //         .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
  //         .whileActiveContinuous(
  //             CargoHandlingCommandBuilder.getShooterAutoSetCommand(
  //                 m_shooter,
  //                 m_drivetrain::getDistanceToHubMeters,
  //                 m_drivetrain::getFieldRelativeVelocityMetersPerSecond));

  //     // Feed to shooter on operator right bumper: waits until shooter at setpoint
  //     m_operator
  //         .rightBumper()
  //         .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
  //         .whileActiveContinuous(
  //             CargoHandlingCommandBuilder.getIndexToShooterCommand(
  //                 m_indexer, m_feeder, m_shooter, m_ballCounter));
  //   }

  private void configureClimbCommands() {
    // Climb commands will only be triggered when in climb mode (see ClimbStateMachine)
    // Climb mode on: start and back buttons
    // Climb mode off: left pov and back buttons
    // All climb commands within climb mode use the same control layout:
    // A button: proceed
    // B button: repeat/go back
    // Y button: finish
    // However, not all checkpoints have either repeat or finish options, and none have both.

    // turn on climb mode
    m_driver
        .start()
        .and(m_driver.back())
        .whileActiveOnce(
            new InstantCommand(m_climbStateMachine::enableClimb)
                .alongWith(
                    new InstantCommand(() -> m_climbElevator.enable(true)),
                    new InstantCommand(() -> m_climbArm.enable(true)),
                    ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator)));

    // turn off climb mode
    m_driver
        .leftPOV()
        .and(m_driver.back())
        .whileActiveOnce(
            new InstantCommand(m_climbStateMachine::disableClimb)
                .alongWith(
                    new InstantCommand(() -> m_climbElevator.enable(false)),
                    new InstantCommand(() -> m_climbArm.enable(false)),
                    ClimbCommandBuilder.getEngageRatchetCommand(m_climbElevator)));

    // disengage ratchet
    //
    m_driver.x().whenPressed(ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator));

    // When at pre-climb state, pressing proceed will disengage ratchet and raise hooks.
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kPreClimb))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kAligningToMidbar)
                .andThen(
                    ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator),
                    ClimbCommandBuilder.getRaiseHookCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointAlignedToMidBar)));

    // When at aligned to mid bar checkpoint, pressing proceed will lower hooks onto mid bar.
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointAlignedToMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingHookToMidBar)
                .andThen(
                    ClimbCommandBuilder.getHookOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointHookedOnMidBar)));

    // When at hooked on mid bar checkpoint, pressing proceed will climb on the mid bar.
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kLiftingOnMidBar)
                .andThen(
                    ClimbCommandBuilder.getLiftOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar)));

    // When at hooked on mid bar checkpoint, pressing repeat will move back to pre-climb state

    // driver can try hooking onto mid bar again).
    m_driver
        .x()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(m_climbStateMachine.getSetStateCommand(ClimbState.kPreClimb));

    // when at climbed on mid bar checkpoint, pressing proceed will move arms to high bar
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingArmsToHighBar)
                .andThen(
                    ClimbCommandBuilder.getArmOnNextBarCommand(m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointArmsOnHighBar)));

    // When at climbed on mid bar checkpoint, pressing finish will complete climb on mid bar
    m_driver
        .a()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnMidBar)
                .andThen(
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnMidBar)));

    // When at arms on high bar checkpoint, pressing proceed will pull back arms and translate
    // the bar, then hook on the high bar
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointArmsOnHighBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingToHighBar)
                .andThen(
                    ClimbCommandBuilder.getRollToNextBarAndHookOnCommand(
                        m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointHookedOnHighBar)));

    // When at arms on high bar checkpoint, pressing repeat will move to climbed on mid bar
    // checkpoint.
    m_driver
        .x()
        .and(
            m_climbStateMachine
                .getClimbStateTrigger(ClimbState.kCheckpointArmsOnHighBar)
                .or(m_climbStateMachine.getClimbStateTrigger(ClimbState.kMovingArmsToHighBar)))
        .whenActive(m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar));

    // When at hooked on high bar checkpoint, pressing proceed will reset climber for climb on
    // traverse bar and move arms there
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnHighBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kResettingClimberOnHighBar)
                .andThen(
                    ClimbCommandBuilder.getResetClimberCommand(m_climbElevator, m_climbArm),
                    ClimbCommandBuilder.getArmOnNextBarCommand(m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(
                        ClimbState.kCheckpointArmsOnTraverseBar)));

    // When at hooked on high bar checkpoint, pressing finish will complete climb on high bar
    m_driver
        .a()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnHighBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnHighBar)
                .andThen(
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(m_climbArm, m_climbElevator),
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnHighBar)));

    // When at arms on traverse bar checkpoint, pressing proceed will pull back arms and

    // up
    // the bar, then hook on traverse bar
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointArmsOnTraverseBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingToTraverseBar)
                .andThen(
                    ClimbCommandBuilder.getRollToNextBarAndHookOnCommand(
                        m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(
                        ClimbState.kCheckpointHookedOnTraverseBar)));

    // When at arms on traverse bar checkpoint, pressing repeat will move to hooked on high bar
    // checkpoint.
    m_driver
        .x()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointArmsOnTraverseBar))
        .whenActive(m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointHookedOnHighBar));

    // When at hooked on traverse bar checkpoint, pressing proceed will finish climb
    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnTraverseBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnTraverseBar)
                .andThen(
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(m_climbArm, m_climbElevator),
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnTraverseBar)));

    // m_driver
    //     .x()
    //     .and(
    //         new Trigger(
    //             () -> {
    //               return m_climbMode == true && !m_climbElevator.getHookedOnMidBar();
    //             }))
    //     .whenActive((
    //       () ->  ClimbCommandBuilder.getRaiseHookCommand(m_climbElevator).andThen(() ->
    // m_climbElevator.setHookedOnMidBar(false))));
    // */
    // m_driver
    //     .a()
    //     .and(
    //         new Trigger(
    //             () -> {
    //               return m_climbMode == true;
    //             }))
    //     .whenActive(
    //         m_climbStateMachine.runCurrentStep(
    //             m_climbElevator, m_climbArm, () -> m_driver.b().get(), () ->
    // m_driver.x().get()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // return m_autoChooser.getSelected().andThen(AutoCommandBuilder.setLEDsAutoCommand(m_vision));
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    m_drivetrain.drive(0, 0, 0, false);
  }
}
