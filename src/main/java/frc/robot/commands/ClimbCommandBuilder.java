package frc.robot.commands;

import static frc.robot.Constants.ClimbArmConstants.*;
import static frc.robot.Constants.ClimbElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbArm;
import frc.robot.subsystems.climb.ClimbElevator;
import java.util.function.BooleanSupplier;

public class ClimbCommandBuilder {

  public static Command getMidBarNoProceedClimbCommand(
      ClimbElevator elevator,
      ClimbArm arm,
      BooleanSupplier proceedButton,
      BooleanSupplier stopButton) {
    return getDisengageRatchetCommand(elevator)
        .andThen(
            new InstantCommand(() -> elevator.setMode(false), elevator),
            getRaiseHookCommand(elevator),
            new WaitUntilCommand(proceedButton),
            getHookOnMidBarCommand(elevator),
            new WaitUntilCommand(proceedButton),
            new InstantCommand(
                () -> {
                  elevator.setHookedOnMidBar(true);
                }),
            new InstantCommand(() -> elevator.setMode(true), elevator),
            getLiftOnMidBarCommand(elevator),
            getEngageRatchetAndLowerCommand(elevator));
  }

  // disengage ratchet via servo to allow climb
  public static Command getDisengageRatchetCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setRatchet(false), elevator);
  }

  // Start lined up (roughly) to tape. Raise hook.
  public static Command getRaiseHookCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setMode(false), elevator)
        .andThen(getMoveElevatorCommand(kElevatorSlotSensorTopPosition, elevator));
  }

  // after raise hook command, drivers drive robot back to contact bar, then press proceed
  // button to lower hook to be on bar, which is another driver checkpoint.
  public static Command getHookOnMidBarCommand(ClimbElevator elevator) {
    return getMoveElevatorCommand(Units.inchesToMeters(18.5), elevator);
  }

  // Pull hook down to raise robot. Triggered by driver presssing proceed button. After this driver
  // can choose to proceed to high bar or climb on mid bar.
  public static Command getLiftOnMidBarCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setMode(true), elevator)
        .andThen(getMoveElevatorCommand(Units.inchesToMeters(3.25), elevator));
  }

  // Swing arm up, and then reset absolute rotation.
  public static Command getArmOnNextBarCommand(ClimbElevator elevator, ClimbArm arm) {
    return new RunCommand(() -> arm.setVoltage(-7))
        .withTimeout(2)
        .andThen(new InstantCommand(() -> arm.setVoltage(0)));
  }

  // Reset arm rotation. Pull arm back to settle on bar, unload hook, roll up rack partway, lower
  // hook to fit under bar,
  // and then roll past bar. Then
  // raise hook, roll back a bit, and lower hook to hook on bar.
  public static Command getRollToNextBarAndHookOnCommand(ClimbElevator elevator, ClimbArm arm) {
    return new InstantCommand(() -> arm.resetAbsoluteRotation(0), arm)
        .andThen(
            new InstantCommand(() -> arm.setMode(true), arm),
            getTranslateArmCommand(
                Units.inchesToMeters(0.64901028), arm), // pull arm back to settle on bar
            new InstantCommand(() -> elevator.setMode(false), elevator),
            getMoveElevatorCommand(Units.inchesToMeters(5.75), elevator), // unload hook
            getTranslateArmCommand(0.656, arm)
                .alongWith(
                    new WaitUntilCommand(
                            () -> arm.getTranslationMeters() > Units.inchesToMeters(10))
                        .andThen(
                            getMoveElevatorCommand(
                                kElevatorSlotSensorBottomPosition, elevator))), // lower hook
            new PrintCommand("arm to the end"),
            getMoveElevatorCommand(Units.inchesToMeters(5.5), elevator), // raise hook
            new PrintCommand("elevator up"),
            getTranslateArmCommand(0.6, arm), // roll back
            getMoveElevatorCommand(Units.inchesToMeters(4), elevator)); // lower hook onto bar
  }

  public static Command getResetClimberCommand(ClimbElevator elevator, ClimbArm arm) {
    return new InstantCommand(() -> elevator.setMode(true), elevator)
        .andThen(
            getMoveElevatorCommand(Units.inchesToMeters(3), elevator),
            new InstantCommand(() -> arm.setMode(false), arm),
            getTranslateArmCommand(Units.inchesToMeters(8.125), arm), // push arm forward
            getMoveElevatorCommand(
                Units.inchesToMeters(12), elevator), // extend elevator to release arm
            new InstantCommand(() -> arm.setVoltage(0)), // let arm fall to rest
            new WaitCommand(3), // wait for a few seconds
            new RunCommand(() -> arm.setVoltage(-0.2 * Constants.kNominalVoltage))
                .until(() -> arm.getStatorCurrentSpike(10)),
            new InstantCommand(() -> arm.setVoltage(0)),
            getMoveElevatorCommand(Units.inchesToMeters(3.5), elevator)); // climb vertically
  }

  // Push rack forward a bit to ensure not contacting low bar
  public static Command getPushArmForwardAtEndCommand(ClimbArm arm, ClimbElevator elevator) {
    return getMoveElevatorCommand(Units.inchesToMeters(3.5), elevator)
        .andThen(
            new RunCommand(() -> arm.setVoltage(-0.2 * Constants.kNominalVoltage)).withTimeout(1.5),
            new InstantCommand(() -> arm.setVoltage(0))); // push arm forward some arbitrary amount
  }

  // Engage the ratchet to stop climb sliding down, and then cut power to the elevator
  public static Command getEngageRatchetAndLowerCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setRatchet(true), elevator)
        .andThen(new InstantCommand(elevator::zeroOverride, elevator));
  }

  private static Command getMoveElevatorCommand(double positionMeters, ClimbElevator elevator) {
    return new RunCommand(() -> elevator.setPositionMeters(positionMeters), elevator)
        .withInterrupt(
            () ->
                atGoal(
                    positionMeters,
                    kElevatorPositionToleranceMeters,
                    elevator.getPositionMeters()));
  }

  private static Command getTranslateArmCommand(double translationMeters, ClimbArm arm) {
    return new RunCommand(() -> arm.setTranslationMeters(translationMeters), arm)
        .withInterrupt(
            () ->
                atGoal(
                    translationMeters, kArmTranslationToleranceMeters, arm.getTranslationMeters()));
  }

  private static boolean atGoal(double goal, double tolerance, double measurement) {
    return Math.abs(measurement - goal) < tolerance;
  }
}
