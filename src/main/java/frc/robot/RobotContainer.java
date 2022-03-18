// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final Hood m_hood = new Hood();
  private final Turret m_turret = new Turret();

  private final Drivetrain m_drivetrain =
      new Drivetrain(
          () -> {
            return new Rotation2d();
          });

  public final ClimbElevator m_climbElevator = new ClimbElevator();
  public final ClimbArm m_climbArm = new ClimbArm();
  public final ClimbStateMachine m_climbStateMachine = new ClimbStateMachine();

  // SHOOTER SETPOINT FIELDS
  // Distance offset to change distance by for auto-aim -- used to adjust

  // OPERATOR ADJUSTMENTS
  @Log(name = "Distance offset", width = 4, height = 2, rowIndex = 4, columnIndex = 12)
  double m_distanceOffset = 0;

  @Log(name = "Turret shift", width = 4, height = 2, rowIndex = 6, columnIndex = 12)
  double m_turretShift = 0;

  // OPERATOR LOCKS
  @Log(name = "Last setpoint lock", width = 3, height = 2, rowIndex = 0, columnIndex = 12)
  boolean m_setpointLock = false;

  @Log(name = "2.5 meter lock", width = 3, height = 2, rowIndex = 2, columnIndex = 12)
  boolean m_fixedSetpoint = false;

  @Log(name = "Shooter velocity lock value", width = 4, height = 2, rowIndex = 0, columnIndex = 15)
  double m_shooterVelocityLock = 0;

  @Log(name = "Hood angle lock value", width = 4, height = 2, rowIndex = 2, columnIndex = 15)
  double m_hoodAngleLock = 0;

  @Log(name = "Bar to climb to", width = 2, height = 2, rowIndex = 2, columnIndex = 0)
  int m_barToClimbTo = 0; // 1 mid, 2 high, 3 traverse

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_vision.setTranslationConsumer(m_drivetrain::addVisionMeasurement);
    configureButtonBindings();

    Shuffleboard.getTab("RobotContainer").add(CameraServer.startAutomaticCapture());
  }

  /**
   * Use this method to define your utton->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.configureDrivetrainCommands();
    // this.configureVisionCommands();
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

  private void configureVisionCommands() {}

  private void configureCargoHandlingCommands() {
    // Control systems for hood and shooter enabled when not climbing
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kNotClimbing)
        .whenActive(new InstantCommand(() -> m_shooter.enable(true), m_shooter))
        .whenInactive(new InstantCommand(() -> m_shooter.enable(false), m_shooter));

    // // // Default commands for turret and hood are to auto-aim based on robot pose/distance
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kNotClimbing)
        .whenActive(new InstantCommand(() -> m_turret.enable(true), m_turret))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.getTurretAutoAimCommand(
                m_turret, m_drivetrain::getTurretCenterPoseMeters, () -> -m_turretShift))
        .whenInactive(
            new InstantCommand(() -> m_turret.setAngleRadians(0), m_turret)
                .andThen(
                    new WaitUntilCommand(m_turret::atGoal),
                    new InstantCommand(() -> m_turret.enable(false), m_turret)));

    // Puts intake arm down at start of climb
    m_climbStateMachine
        .getClimbStateTrigger(ClimbState.kNotClimbing)
        .whenInactive(new InstantCommand(() -> m_intakeArm.setPosition(false), m_intakeArm));

    // Intake on driver right trigger: put intake arm down, then run intake and low indexer
    // stopped by hitting high indexer path
    m_driver
        .getJoystickAxisButton(Axis.kRightTrigger, kTriggerThreshold)
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.getIntakeCommand(
                m_intake, m_intakeArm, m_indexer, m_feedServo),
            false)
        .whenInactive(
            new InstantCommand(
                    () ->
                        m_indexer.setVelocityRotationsPerSecond(
                            m_indexer.getVelocityRotationsPerSecond()),
                    m_indexer)
                .andThen(
                    new WaitCommand(0.5),
                    CargoHandlingCommandBuilder.getStopIntakeCommand(
                        m_intake, m_intakeArm, m_indexer)));

    // Set shooter on operator left trigger: based on distance to hub
    m_operator
        .getJoystickAxisButton(Axis.kLeftTrigger, kTriggerThreshold)
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whileActiveContinuous(
            new ConditionalCommand(
                CargoHandlingCommandBuilder.getShooterAutoSetCommand(
                    m_shooter,
                    () ->
                        m_fixedSetpoint
                            ? kShootingSetpointDistance
                            : m_drivetrain.getHubToTurretCenterDistanceMeters(),
                    () -> -m_distanceOffset),
                CargoHandlingCommandBuilder.getShooterSetCommand(
                    m_shooter, () -> m_shooterVelocityLock),
                () -> !m_setpointLock))
        .whenInactive(CargoHandlingCommandBuilder.getShooterOffCommand(m_shooter));

    // Feed to shooter on operator right bumper: waits until shooter at setpoint
    m_operator
        .rightBumper()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(
                m_indexer, m_feeder, m_feedServo, m_shooter))
        .whenActive(
            new ConditionalCommand(
                CargoHandlingCommandBuilder.getHoodAutoAimCommand(
                    m_hood,
                    () ->
                        m_fixedSetpoint
                            ? kShootingSetpointDistance
                            : m_drivetrain.getHubToTurretCenterDistanceMeters(),
                    () -> --m_distanceOffset),
                CargoHandlingCommandBuilder.getHoodSetCommand(m_hood, () -> m_hoodAngleLock),
                () -> !m_setpointLock))
        .whenInactive(
            CargoHandlingCommandBuilder.getStopFeedCommand(m_indexer, m_feeder, m_feedServo)
                .alongWith(CargoHandlingCommandBuilder.getHoodDownCommand(m_hood)));

    // Eject on operator X button
    m_operator
        .x()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whileActiveContinuous(
            CargoHandlingCommandBuilder.getEjectCommand(
                m_shooter,
                m_hood,
                m_turret,
                m_indexer,
                m_feeder,
                m_feedServo,
                m_drivetrain::getPoseMeters))
        .whenInactive(
            CargoHandlingCommandBuilder.getStopFeedCommand(m_indexer, m_feeder, m_feedServo)
                .alongWith(
                    CargoHandlingCommandBuilder.getHoodDownCommand(m_hood),
                    CargoHandlingCommandBuilder.getShooterOffCommand(m_shooter)));

    // Make robot think it's closer when aiming
    m_operator
        .upperPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_distanceOffset += 0.2;
                }));

    // Make robot think it's further when aiming
    m_operator
        .lowerPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_distanceOffset -= 0.2;
                }));

    // Shift turret to right
    m_operator
        .rightPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_turretShift += 0.05;
                }));

    // Shift turret to left
    m_operator
        .leftPOV()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kNotClimbing))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_turretShift -= 0.05;
                }));

    // lock current shooter setpoints in place
    m_operator
        .b()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_setpointLock = true;
                  m_shooterVelocityLock =
                      CargoHandlingCommandBuilder.getShooterAutoSetpoint(
                              () -> m_drivetrain.getHubToTurretCenterDistanceMeters(),
                              () -> m_distanceOffset)
                          .getAsDouble();
                  m_hoodAngleLock =
                      CargoHandlingCommandBuilder.getHoodAutoSetpoint(
                              () -> m_drivetrain.getHubToTurretCenterDistanceMeters(),
                              () -> m_distanceOffset)
                          .getAsDouble();
                }))
        .whenInactive(
            new InstantCommand(
                () -> {
                  m_setpointLock = false;
                }));

    m_operator
        .y()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_fixedSetpoint = true;
                  m_shooterVelocityLock =
                      CargoHandlingCommandBuilder.getShooterAutoSetpoint(
                              () -> Constants.kShootingSetpointDistance, () -> 0)
                          .getAsDouble();
                  m_hoodAngleLock =
                      CargoHandlingCommandBuilder.getHoodAutoSetpoint(
                              () -> Constants.kShootingSetpointDistance, () -> 0)
                          .getAsDouble();
                }))
        .whenInactive(new InstantCommand(() -> m_fixedSetpoint = false));

    // reset all
    m_operator
        .back()
        .whenActive(
            new InstantCommand(
                () -> {
                  m_fixedSetpoint = false;
                  m_setpointLock = false;
                  m_distanceOffset = 0;
                  m_turretShift = 0;
                  m_shooterVelocityLock = 0;
                  m_hoodAngleLock = 0;
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
                    new InstantCommand(() -> m_climbArm.enable(true), m_climbArm))
                .andThen(ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator)));

    // turn off climb mode
    m_driver
        .leftPOV()
        .and(m_driver.back())
        .whileActiveOnce(
            new InstantCommand(m_climbStateMachine::disableClimb)
                .alongWith(
                    new InstantCommand(() -> m_climbElevator.enable(false), m_climbElevator),
                    new InstantCommand(() -> m_climbArm.enable(false), m_climbArm))
                .andThen(ClimbCommandBuilder.getEngageRatchetCommand(m_climbElevator)));

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
                    ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator)
                        .alongWith(ClimbCommandBuilder.getDisengageArmCommand(m_climbArm)),
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

    m_driver
        .y()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_barToClimbTo = 3;
                }));

    m_driver
        .b()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_barToClimbTo = 2;
                }));

    m_driver
        .a()
        .and(m_climbStateMachine.getClimbStateTrigger(ClimbState.kCheckpointLiftedOnMidBar))
        .whenActive(
            new InstantCommand(
                () -> {
                  m_barToClimbTo = 1;
                }));

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
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(m_climbArm, m_climbElevator),
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
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
                    ClimbCommandBuilder.getPushArmForwardAtEndCommand(m_climbArm, m_climbElevator),
                    ClimbCommandBuilder.getEngageRatchetAndLowerCommand(m_climbElevator),
                    m_climbStateMachine.getSetStateCommand(ClimbState.kClimbedOnTraverseBar)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto =
        AutoCommandBuilder.get4BallAuto(
                m_drivetrain,
                m_intake,
                m_intakeArm,
                m_indexer,
                m_feeder,
                m_feedServo,
                m_shooter,
                m_hood)
            .andThen(
                AutoCommandBuilder.getFinishAutoCommand(
                    m_intake, m_intakeArm, m_indexer, m_feeder, m_feedServo, m_shooter, m_hood));

    return auto;
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    // m_drivetrain.drive(0, 0, 0, false);
    m_feeder.setVoltage(0);
  }
}
