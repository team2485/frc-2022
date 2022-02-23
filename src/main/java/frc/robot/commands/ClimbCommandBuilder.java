package frc.robot.commands;

import static frc.robot.Constants.ClimbArmConstants.*;
import static frc.robot.Constants.ClimbElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climb.ClimbArm;
import frc.robot.subsystems.climb.ClimbElevator;
import java.util.function.BooleanSupplier;

public class ClimbCommandBuilder {

  public static Command lineUpToTape() {
    return () -> {
      System.out.println("one");
      return null;
    };
    // ClimbStateMachine.setState(3);
  }

  public static Command raiseHook() {
    return () -> {
      System.out.println("two");
      return null;
    };
  }

  public static Command backUp() {
    return () -> {
      System.out.println("three");
      return null;
    };
  }

  public static Command hookOnBar() {
    return () -> {
      System.out.println("four");
      return null;
    };
  }

  public static Command climbVertically() {
    return () -> {
      System.out.println("five");
      return null;
    };
  }

  public static Command swingRackUp() {
    return () -> {
      System.out.println("six");
      return null;
    };
  }

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
            getMidBarLiftCommand(elevator),
            getEngageRatchetAndLowerCommand(elevator));
  }

  public static Command getMidBarClimbCommand(
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
            getMidBarLiftCommand(elevator),
            new WaitUntilCommand(
                () -> {
                  return proceedButton.getAsBoolean() || stopButton.getAsBoolean();
                }),
            new ConditionalCommand(
                getMidToHighBarCommand(elevator, arm, proceedButton, stopButton),
                getEngageRatchetAndLowerCommand(elevator),
                () -> {
                  return proceedButton.getAsBoolean() && !stopButton.getAsBoolean();
                }));
  }

  public static Command getMidToHighBarCommand(
      ClimbElevator elevator,
      ClimbArm arm,
      BooleanSupplier proceedButton,
      BooleanSupplier stopButton) {
    return getArmOnNextBarCommand(arm)
        .andThen(
            getRollToNextBarCommand(elevator, arm),
            getHookOnNextBarCommand(elevator, arm),
            getPinArmOnBarCommand(elevator),
            new WaitUntilCommand(
                () -> {
                  return proceedButton.getAsBoolean() || stopButton.getAsBoolean();
                }),
            new ConditionalCommand(
                getHighToTraverseBarCommand(elevator, arm),
                getPushArmForwardAtEndCommand(arm)
                    .andThen(getEngageRatchetAndLowerCommand(elevator)),
                () -> {
                  return proceedButton.getAsBoolean() && !stopButton.getAsBoolean();
                }));
  }

  public static Command getHighToTraverseBarCommand(ClimbElevator elevator, ClimbArm arm) {
    return getPushArmForwardCommand(arm)
        .andThen(
            getDropArmCommand(elevator),
            getLowerArmCommand(arm),
            getClimbOnNextBarCommand(elevator),
            getArmOnNextBarCommand(arm),
            getRollToNextBarCommand(elevator, arm),
            getHookOnNextBarCommand(elevator, arm),
            getPinArmOnBarCommand(elevator),
            getPushArmForwardAtEndCommand(arm),
            getEngageRatchetAndLowerCommand(elevator));
  }

  // disengage ratchet via servo to allow climb
  public static Command getDisengageRatchetCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setRatchet(false), elevator);
  }

  // Start lined up (roughly) to tape. Raise hook.
  public static Command getRaiseHookCommand(ClimbElevator elevator) {
    return new RunCommand(
            () -> elevator.setPositionMeters(kElevatorSlotSensorTopPosition), elevator)
        .withInterrupt(
            () ->
                atGoal(
                    kElevatorSlotSensorTopPosition,
                    kElevatorPositionToleranceMeters,
                    elevator.getPositionMeters()));
  }

  // after raise hook command, drivers drive robot back to contact bar, then press proceed
  // button. After button press, this lowers hook to be on bar, which is another driver chekpoint.
  public static Command getHookOnMidBarCommand(ClimbElevator elevator) {
    return new RunCommand(() -> elevator.setPositionMeters(Units.inchesToMeters(18.5)), elevator)
        .withInterrupt(
            () ->
                atGoal(
                    Units.inchesToMeters(18.5),
                    kElevatorPositionToleranceMeters,
                    elevator.getPositionMeters()));
  }

  // Pull hook down to raise robot. Triggered by driver presssing proceed button. After this driver
  // can choose to proceed to high bar or climb on mid bar.
  public static Command getMidBarLiftCommand(ClimbElevator elevator) {
    return new RunCommand(() -> elevator.setPositionMeters(Units.inchesToMeters(3.5)), elevator)
        .withInterrupt(
            () ->
                atGoal(
                    Units.inchesToMeters(3.5),
                    kElevatorPositionToleranceMeters,
                    elevator.getPositionMeters()));
  }

  // Swing arm up, and then reset absolute rotation.
  public static Command getArmOnNextBarCommand(ClimbArm arm) {
    return new RunCommand(() -> arm.setVoltage(3))
        .withInterrupt(
            () -> atGoal(-0.340666666667, kArmRotationTolerance, arm.getAbsoluteRotation()))
        .andThen(new InstantCommand(() -> arm.resetAbsoluteRotation(0)));
  }

  // pull arm back to settle on bar
  public static Command getPullArmBackCommand(ClimbArm arm) {
    return new RunCommand(() -> arm.setTranslationMeters(Units.inchesToMeters(0.64901028)))
        .withInterrupt(
            () ->
                atGoal(
                    Units.inchesToMeters(0.64901028),
                    kArmTranslationToleranceMeters,
                    arm.getTranslationMeters()));
  }

  // Unload hook, roll up rack partway, lower hook to fit under bar, and then roll past bar.
  public static Command getRollToNextBarCommand(ClimbElevator elevator, ClimbArm arm) {
    return null;
  }

  // Raise hook, roll back a bit, and then lower hook to hook on bar.
  public static Command getHookOnNextBarCommand(ClimbElevator elevator, ClimbArm arm) {
    return null;
  }

  // Lower hook down to engage with bar, then further down to pin rack against bar.
  public static Command getPinArmOnBarCommand(ClimbElevator elevator) {
    return null;
  }

  // Push rack arms forward while pinned on bar.
  public static Command getPushArmForwardCommand(ClimbArm arm) {
    return null;
  }

  // Unpin arm and let it drop and swing.
  public static Command getDropArmCommand(ClimbElevator elevator) {
    return null;
  }

  // Lower rack arm to reset position
  public static Command getLowerArmCommand(ClimbArm arm) {
    return null;
  }

  // Pull robot up via elevator
  public static Command getClimbOnNextBarCommand(ClimbElevator elevator) {
    return null;
  }

  // Push rack forward a bit to ensure not contacting low bar
  public static Command getPushArmForwardAtEndCommand(ClimbArm arm) {
    return null;
  }

  // Engage the ratchet to stop climb sliding down, and then cut power to the elevator
  public static Command getEngageRatchetAndLowerCommand(ClimbElevator elevator) {
    return new InstantCommand(() -> elevator.setRatchet(true), elevator)
        .andThen(new InstantCommand(elevator::zeroOverride, elevator));
  }

  public static boolean atGoal(double goal, double tolerance, double measurement) {
    return Math.abs(measurement - goal) < tolerance;
  }
}
