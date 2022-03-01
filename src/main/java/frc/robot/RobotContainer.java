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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climb.ClimbArm;
import frc.robot.subsystems.climb.ClimbElevator;
import frc.robot.subsystems.climb.ClimbStateMachine;
import frc.robot.subsystems.climb.ClimbStateMachine.ClimbState;
import frc.team2485.WarlordsLib.oi.CommandXboxController;
import io.github.oblarg.oblog.annotations.*;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(kDriverPort);
  private final CommandXboxController m_operator = new CommandXboxController(kOperatorPort);
  //   Flywheel m_flywheel = new Flywheel();

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Vision m_vision = new Vision();

  public final ClimbElevator m_climbElevator = new ClimbElevator();
  public final ClimbArm m_climbArm = new ClimbArm();
  public final ClimbStateMachine m_climbStateMachine = new ClimbStateMachine();

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
    this.configureClimbCommands();
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
                .alongWith(ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator)));

    // turn off climb mode
    m_driver
        .leftPOV()
        .and(m_driver.back())
        .whileActiveOnce(
            new InstantCommand(m_climbStateMachine::disableClimb)
                .alongWith(ClimbCommandBuilder.getEngageRatchetCommand(m_climbElevator)));

    // disengage ratchet
    // m_driver.x().whenPressed(ClimbCommandBuilder.getDisengageRatchetCommand(m_climbElevator));

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

    // When at hooked on mid bar checkpoint, pressing repeat will move back to pre-climb state (so
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

    // When at arms on high bar checkpoint, pressing proceed will pull back arms and translate up
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

    // When at arms on traverse bar checkpoint, pressing proceed will pull back arms and translate
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

    return null;
  }

  // whenever the robot is disabled, drive should be turned off
  public void disabledInit() {
    m_drivetrain.drive(0, 0, 0, false);
  }
}
