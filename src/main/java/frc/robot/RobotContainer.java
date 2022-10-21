// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoCommandBuilder;
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
  private final FeedServo m_feedServo = new FeedServo();
  public final Shooter m_shooter = new Shooter();

  public final Drivetrain m_drivetrain = new Drivetrain();

  public final Hood m_hood = new Hood();

  public final ClimbElevator m_climbElevator = new ClimbElevator();
  public final ClimbArm m_climbArm = new ClimbArm();
  public final ClimbStateMachine m_climbStateMachine = new ClimbStateMachine();
  public WPI_TalonFX m_talon = new WPI_TalonFX(40);

  // SHOOTER SETPOINT FIELDS
  // Distance offset to change distance by for auto-aim -- used to adjust

  // OPERATOR ADJUSTMENTS
  @Log(name = "Distance offset", width = 4, height = 1, rowIndex = 4, columnIndex = 16)
  double m_shooterOffset = 0;

  @Log(name = "Kicker ratio offset", width = 4, height = 1, rowIndex = 5, columnIndex = 16)
  double m_kickerRatioOffset = 0;

  // @Log(name = "Angle shift", width = 2, height = 1, rowIndex = 3, columnIndex = 17)
  double m_angleShift = 0;

  // OPERATOR LOCKS

  boolean m_setpointLock = false;

  @Log(name = "High fender lock", width = 4, height = 2, rowIndex = 0, columnIndex = 15)
  boolean m_highFender = false;

  @Log(name = "Low fender lock", width = 4, height = 2, rowIndex = 2, columnIndex = 15)
  boolean m_lowFender = false;

  double hoodAngle = 0.1;
  double flywheelSpeed = 30;

  // @Log(name = "Eject lock", width = 3, height = 1, rowIndex = 5, columnIndex = 12)
  // boolean m_eject = false;

  //   @Log(name = "Shooter velocity lock value", width = 4, height = 1, rowIndex = 0, columnIndex =
  // 15)
  double m_shooterVelocityLock = 0;

  //   @Log(
  //       name = "Shooter tangential ratio lock",
  //       width = 4,
  //       height = 1,
  //       rowIndex = 0,
  //       columnIndex = 15)
  double m_shooterTangentialRatioLock = 1;

  @Log(name = "Bar to climb to", width = 2, height = 2, rowIndex = 2, columnIndex = 0)
  int m_barToClimbTo = 0; // 1 mid, 2 high, 3 traverse

  @Log(name = "Auto Chooser", width = 2, height = 2, rowIndex = 4, columnIndex = 0)
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  Timer m_autoTimer = new Timer();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    m_climbElevator.setPositionMeters(0);

    m_autoChooser.setDefaultOption(
        "4 Ball Right Side",
        AutoCommandBuilder.get4BallAuto(
            m_drivetrain,
            m_intake,
            m_intakeArm,
            m_indexer,
            m_feeder,
            m_feedServo,
            m_shooter,
            m_hood));
    m_autoChooser.addOption(
        "3 Ball Left Side",
        AutoCommandBuilder.get3BallAutoLeft(
            m_drivetrain,
            m_intake,
            m_intakeArm,
            m_indexer,
            m_feeder,
            m_feedServo,
            m_shooter,
            m_hood));

    m_autoChooser.addOption(
        "5 Ball (flex)",
        AutoCommandBuilder.get5BallAuto(
            m_drivetrain,
            m_intake,
            m_intakeArm,
            m_indexer,
            m_feeder,
            m_feedServo,
            m_shooter,
            m_hood));

    m_autoChooser.addOption(
        "Back up",
        // PathCommandBuilder.getResetOdometryCommand(m_drivetrain,
        // PathCommandBuilder.getPathCommand(m_drivetrain, "2 Ball Left Side")).andThen(
        // CargoHandlingCommandBuilder.getSetShooterCommand(
        //         () -> 35, () -> 0.8, m_shooter) // full send
        //     .raceWith(
        //         new WaitCommand(3)
        //             .andThen(
        //                 CargoHandlingCommandBuilder.getIndexToShooterCommand(
        //                     m_indexer, m_feeder, m_feedServo),
        //                 new WaitCommand(1)))
        //     .andThen(
        new RunCommand(() -> m_drivetrain.drive(-1, 0, 0, false)).withTimeout(2.5));
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
    this.configureCargoHandlingCommands();
    this.configureClimbCommands();
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
        .rightStick()
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

  private void configureCargoHandlingCommands() {

    // Puts intake arm down at start of climb

    // Intake on driver right trigger: put intake arm down, then run intake and low indexer
    // stopped by hitting high indexer path

    // m_driver
    //     .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
    //     .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
    //     .whileActiveContinuous(
    //         CargoHandlingCommandBuilder.getIntakeCommand(
    //             m_intake, m_intakeArm, m_indexer, m_feedServo),
    //         true)
    //     .whenInactive(
    //         new InstantCommand(
    //                 () ->
    //                     m_indexer.setVelocityRotationsPerSecond(
    //                         m_indexer.getVelocityRotationsPerSecond()),
    //                 m_indexer)
    //             .andThen(
    //                 new WaitCommand(0.5),
    //                 CargoHandlingCommandBuilder.getStopIntakeCommand(
    //                         m_intake, m_intakeArm, m_indexer)
    //                     .alongWith(
    //                         new StartEndCommand(
    //                                 () -> m_operator.setRumble(RumbleType.kLeftRumble, 0.5),
    //                                 () -> m_operator.setRumble(RumbleType.kLeftRumble, 0))
    //                             .withTimeout(0.5))));

    // m_operator.upperPOV().whileActiveOnce(new InstantCommand(() -> hoodAngle += 0.01));
    // m_operator.lowerPOV().whileActiveOnce(new InstantCommand(() -> hoodAngle -= 0.01));

    // m_operator
    //     .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
    //     .whileActiveOnce(new InstantCommand(() -> m_hood.setAngleRadians(hoodAngle)));
    m_operator.leftBumper().whileActiveOnce(new InstantCommand(() -> m_hood.setAngleRadians(0)));

    // m_operator.leftPOV().whileActiveOnce(new InstantCommand(() -> flywheelSpeed--));
    // m_operator.rightPOV().whileActiveOnce(new InstantCommand(() -> flywheelSpeed++));

    m_operator.y().whenActive(CargoHandlingCommandBuilder.setShooterForShot(m_hood, m_shooter));

    //  m_operator.y().whenActive(new InstantCommand(()->m_hood.setAngleRadians(0.16)));

    m_operator
        .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
        .whileActiveContinuous(CargoHandlingCommandBuilder.allignToHub(m_drivetrain));

    m_driver
        .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
        // .and(m_climbStateMachine.getClimbStateTrigger((ClimbState.kNotClimbing)))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.runTestCommand(m_intake, m_intakeArm, m_indexer))
        .whenInactive(CargoHandlingCommandBuilder.stopTestCommand(m_intake, m_intakeArm, m_indexer));

    m_driver
        .b()
        // .and(m_climbStateMachine.getClimbStateTrigger((ClimbState.kNotClimbing)))
        .whileActiveContinuous(CargoHandlingCommandBuilder.outtakeCommand(m_intake, m_intakeArm))
        .whenInactive(
            CargoHandlingCommandBuilder.stopTestCommand(m_intake, m_intakeArm, m_indexer));

    m_operator
        .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
        // .and(m_climbStateMachine.getClimbStateTrigger((ClimbState.kNotClimbing)))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.getSetShooterCommand(m_shooter)
                .alongWith(
                    new ConditionalCommand(
                        new InstantCommand(() -> m_operator.setRumble(RumbleType.kLeftRumble, 0.5)),
                        new InstantCommand(()->m_operator.setRumble(RumbleType.kLeftRumble, 0)),
                        () -> m_shooter.shooterWithinTolerance())));

    m_operator
        .rightBumper()
        // .and(m_climbStateMachine.getClimbStateTrigger((ClimbState.kNotClimbing)))
        .whileActiveContinuous(CargoHandlingCommandBuilder.getRunFeederCommand(m_feeder, m_indexer))
        .whenInactive(CargoHandlingCommandBuilder.getStopFeederCommand(m_feeder, m_indexer));

    // m_driver
    //     .upperPOV()
    //     .whileActiveContinuous(CargoHandlingCommandBuilder.getArmUpCommand(m_intakeArm));

    // m_operator
    //     .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
    //     .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
    //     .whenActive(
    //         new InstantCommand(
    //             () ->
    //                 m_indexer.setVelocityRotationsPerSecond(
    //                     Constants.IndexerConstants.kIndexerDefaultSpeedRotationsPerSecond),
    //             m_indexer))
    //     .whenInactive(
    //         new InstantCommand(() -> m_indexer.setVelocityRotationsPerSecond(0), m_indexer));

    // Set shooter on operator left trigger: based on distance to hub
    // m_operator
    //     .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
    //     .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
    //     .whileActiveContinuous(
    //         new ConditionalCommand(
    //             new InstantCommand(),
    //             CargoHandlingCommandBuilder.getSetShooterCommand(
    //                 () -> m_shooterVelocityLock + m_shooterOffset,
    //                 m_shooter),
    //             () -> !m_setpointLock));

    // Feed to shooter on operator right bumper: waits until shooter at setpoint
    // m_operator
    //     .rightBumper()
    //     .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
    //     .whileActiveContinuous(
    //         CargoHandlingCommandBuilder.getIndexToShooterCommand(m_indexer, m_feeder,
    // m_feedServo),
    //         false)
    //     .whenInactive(
    //         CargoHandlingCommandBuilder.getStopFeedCommand(m_indexer, m_feeder, m_feedServo));

    m_operator
        .upperPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_shooterOffset += 0.3;
                  m_kickerRatioOffset += 0.1;
                }));

    // Make robot think it's further when aiming
    m_operator
        .lowerPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_shooterOffset -= 0.3;
                  m_kickerRatioOffset -= 0.1;
                }));

    m_operator
        .a()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_lowFender = true;
                  m_setpointLock = true;
                  m_shooterVelocityLock = 6;
                  m_shooterTangentialRatioLock = 3.2;
                }))
        .whenInactive(
            new InstantCommand(
                () -> {
                  m_lowFender = false;
                  m_setpointLock = false;
                }));

    m_operator
        .y()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_highFender = true;
                  m_setpointLock = true;
                  m_shooterVelocityLock = kShootingFenderSetpointShooter;
                  m_shooterTangentialRatioLock = kShootingFenderSetpointTangentialRatio;
                }))
        .whenInactive(
            new InstantCommand(
                () -> {
                  m_highFender = false;
                  m_setpointLock = false;
                }));

    // reset all
    m_operator
        .back()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_highFender = false;
                  m_setpointLock = false;
                  m_shooterOffset = 0;
                  m_angleShift = 0;
                  m_shooterVelocityLock = 0;
                }));
  }

  private void configureClimbCommands() {
    // Climb commands will only be triggered when in climb mode (see ClimbStateMachine)
    // Climb mode on: start and back buttons
    // Climb mode off: left pov and back buttons
    // All climb commands within climb mode use the same control layout:
    // A button: proceed
    // B button: repeat/go back
    // Y button: finish
    // However, not all checkpoints have either repeat or finish options, and none have both.

    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kNotClimbing)
        .whenActive(
            new InstantCommand(
                () -> {
                  m_climbElevator.enable(false);
                  m_climbArm.enable(false);
                  m_climbElevator.setRatchet(true);
                },
                m_climbElevator,
                m_climbArm));
    // turn on climb mode
    m_driver
        .start()
        .and(m_driver.back())
        .whileActiveOnce(
            new InstantCommand(m_climbStateMachine::enableClimb)
                .alongWith(
                    new InstantCommand(() -> m_climbElevator.enable(true), m_climbElevator),
                    new InstantCommand(() -> m_climbArm.enable(true), m_climbArm)));

    // turn off climb mode
    m_driver
        .leftPOV()
        .and(m_driver.back())
        .whenActive(
            new PerpetualCommand(
                new RunCommand(m_climbStateMachine::disableClimb)
                    .alongWith(
                        new RunCommand(
                            () -> {
                              m_climbElevator.enable(false);
                            },
                            m_climbElevator),
                        new RunCommand(
                            () -> {
                              m_climbArm.enable(false);
                            },
                            m_climbArm))),
            false);

    // disengage ratchet
    //

    // When at pre-climb state, pressing proceed will disengage ratchet and raise hooks.

    // m_driver.getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold).whenActive(new
    // InstantCommand(()->m_climbElevator.setPositionMeters(0)));

    m_driver
        .rightBumper()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kPreClimb))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kAligningToMidbar)
                .andThen(
                    ClimbCommandBuilder.getDisengageArmCommand(m_climbArm),
                    ClimbCommandBuilder.getRaiseHookCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointAlignedToMidBar)));

    // When at aligned to mid bar checkpoint, pressing proceed will lower hooks onto mid bar.
    m_driver
        .rightBumper()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointAlignedToMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingHookToMidBar)
                .andThen(
                    ClimbCommandBuilder.getHookOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointHookedOnMidBar)));

    // When at hooked on mid bar checkpoint, pressing proceed will climb on the mid bar.
    m_driver
        .a()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kClimbingOnMidBar)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          m_barToClimbTo = 1;
                        }))
                .andThen(
                    ClimbCommandBuilder.getLiftOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar)));

    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kClimbingOnMidBar)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          m_barToClimbTo = 2;
                        }))
                .andThen(
                    ClimbCommandBuilder.getLiftOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar)));

    m_driver
        .y()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kClimbingOnMidBar)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          m_barToClimbTo = 3;
                        }))
                .andThen(
                    ClimbCommandBuilder.getLiftOnMidBarCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar)));

    // .or(m_driver.b())
    // .or(m_driver.y())
    // .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
    // .whenActive(
    //     m_climbStateMachine
    //         .getSetStateCommand(ClimbState.kClimbingOnMidBar)
    //         .andThen(
    //             ClimbCommandBuilder.getLiftOnMidBarCommand(m_climbElevator),
    //             m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointLiftedOnMidBar)));

    // When at hooked on mid bar checkpoint, pressing repeat will move back to pre-climb state

    // driver can try hooking onto mid bar again).
    m_driver
        .leftBumper()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointHookedOnMidBar))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kAligningToMidbar)
                .andThen(
                    ClimbCommandBuilder.getRaiseHookCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointAlignedToMidBar)));

    // when at climbed on mid bar checkpoint, pressing proceed will move arms to high bar
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar)
        .and(new Trigger(() -> m_barToClimbTo == 2 || m_barToClimbTo == 3))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingArmsToHighBar)
                .andThen(
                    ClimbCommandBuilder.getArmOnNextBarCommand(m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointArmsOnHighBar)));

    // When at climbed on mid bar checkpoint, pressing finish will complete climb on mid bar
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar)
        .and(new Trigger(() -> m_barToClimbTo == 1))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnMidBar)
                .andThen(
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnMidBar)));

    // When at arms on high bar checkpoint, pressing proceed will pull back arms and translate
    // the bar, then hook on the high bar
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointArmsOnHighBar)
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingToHighBar)
                .andThen(
                    ClimbCommandBuilder.getRollToNextBarAndHookOnCommand(
                        m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kCheckpointHookedOnHighBar)));

    // When at hooked on high bar checkpoint, pressing proceed will reset climber for climb on
    // traverse bar and move arms there
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointHookedOnHighBar)
        .and(new Trigger(() -> m_barToClimbTo == 3))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kResettingClimberOnHighBar)
                .andThen(
                    ClimbCommandBuilder.getResetClimberCommand(m_climbElevator, m_climbArm),
                    ClimbCommandBuilder.getArmOnNextBarCommand(m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(
                        ClimbState.kCheckpointArmsOnTraverseBar)));

    // When at hooked on high bar checkpoint, pressing finish will complete climb on high bar
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointHookedOnHighBar)
        .and(new Trigger(() -> m_barToClimbTo == 2))
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnHighBar)
                .andThen(
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(
                        m_climbArm, m_climbElevator, m_intakeArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnHighBar)));

    // When at arms on traverse bar checkpoint, pressing proceed will pull back arms and up
    // the bar, then hook on traverse bar
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointArmsOnTraverseBar)
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kMovingToTraverseBar)
                .andThen(
                    ClimbCommandBuilder.getRollToNextBarAndHookOnCommand(
                        m_climbElevator, m_climbArm),
                    m_climbStateMachine.getSetStateCommand(
                        ClimbState.kCheckpointHookedOnTraverseBar)));

    // When at hooked on traverse bar checkpoint, pressing proceed will finish climb
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kCheckpointHookedOnTraverseBar)
        .whenActive(
            m_climbStateMachine
                .getSetStateCommand(ClimbState.kFinishingClimbOnTraverseBar)
                .andThen(
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(
                        m_climbArm, m_climbElevator, m_intakeArm),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnTraverseBar)));

    // m_climbStateMachine.getClimbStateTrigger(ClimbState.kClimbedOnHighBar).or(
    //     m_climbStateMachine.getClimbStateTrigger(ClimbState.kClimbedOnTraverseBar).whenActive(

    //     )
    // )
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command auto =
    //     AutoCommandBuilder.get2BallAutoLeft(
    //             m_drivetrain, m_intake, m_intakeArm, m_indexer, m_feeder, m_feedServo, m_shooter)
    //         .andThen(
    //             AutoCommandBuilder.getFinishAutoCommand(
    //                 m_intake, m_intakeArm, m_indexer, m_feeder, m_feedServo, m_shooter, m_hood));

    return m_autoChooser.getSelected();

    // WL_SwerveControllerCommand path =
    //     PathCommandBuilder.getPathCommand(m_drivetrain, "3 Score Right Fender");

    // return PathCommandBuilder.getResetOdometryCommand(m_drivetrain, path)
    //     .andThen(
    //         new InstantCommand(
    //             () ->
    // m_drivetrain.getField2d().getObject("traj").setTrajectory(path.m_trajectory),
    //             m_drivetrain),
    //         path);
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    // m_drivetrain.drive(0, 0, 0, false);
    m_feeder.setVoltage(0);
  }
}
