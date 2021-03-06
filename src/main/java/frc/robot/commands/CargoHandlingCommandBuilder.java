package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
import frc.robot.subsystems.cargoHandling.FeedServo;
import frc.robot.subsystems.cargoHandling.Feeder;
import frc.robot.subsystems.cargoHandling.Indexer;
import frc.robot.subsystems.cargoHandling.Intake;
import frc.robot.subsystems.cargoHandling.IntakeArm;
import frc.robot.subsystems.cargoHandling.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CargoHandlingCommandBuilder {

  public static Command getIntakeCommand(
      Intake intake, IntakeArm intakeArm, Indexer indexer, FeedServo servo) {
    return getIntakeArmDownCommand(intakeArm)
        .alongWith(
            new RunCommand(
                    () ->
                        intake.setVelocityRotationsPerSecond(kIntakeDefaultSpeedRotationsPerSecond),
                    intake)
                .alongWith(
                    new RunCommand(
                        () ->
                            indexer.setVelocityRotationsPerSecond(
                                kIndexerIntakeSpeedRatio * kIntakeDefaultSpeedRotationsPerSecond),
                        indexer),
                    new InstantCommand(() -> servo.engage(false), servo)));
  }

  public static Command getStopIntakeCommand(Intake intake, IntakeArm intakeArm, Indexer indexer) {
    return getIntakeArmUpCommand(intakeArm)
        .alongWith(
            new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer),
            new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0), intake));
  }

  public static Command getShooterAutoSetCommand(
      DoubleSupplier distanceToHub, DoubleSupplier distanceOffset, Shooter shooter) {
    return getSetShooterOnlyCommand(getShooterAutoSetpoint(distanceToHub, distanceOffset), shooter);
  }

  public static DoubleSupplier getShooterAutoSetpoint(
      DoubleSupplier distanceToHub, DoubleSupplier distanceOffset) {
    return () ->
        InterpolatingTable.get(distanceToHub.getAsDouble() + distanceOffset.getAsDouble())
            .shooterSpeedRotationsPerSecond;
  }

  public static Command getIndexToShooterCommand(Indexer indexer, Feeder feeder, FeedServo servo) {
    return new ParallelRaceGroup(
            getSetFeederCommand(() -> 50, feeder),
            new WaitCommand(0.2)
                .andThen(getSetServoCommand(() -> true, servo), new WaitCommand(0.4)))
        .andThen(
            getSetFeederCommand(() -> 0, feeder).alongWith(getSetServoCommand(() -> false, servo)),
            new WaitCommand(0.5),
            getSetIndexerCommand(() -> kIndexerDefaultSpeedRotationsPerSecond, indexer)
                .withTimeout(0.8));
  }

  public static Command getStopFeedCommand(Indexer indexer, Feeder feeder, FeedServo servo) {
    return new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer)
        .alongWith(
            new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder),
            new InstantCommand(() -> servo.engage(false), servo));
  }

  public static Command getSetIndexerCommand(DoubleSupplier velocity, Indexer indexer) {
    if (velocity.getAsDouble() == 0) {
      return new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer);
    } else {
      return new StartEndCommand(
          () -> indexer.setVelocityRotationsPerSecond(velocity.getAsDouble()),
          () -> indexer.setVelocityRotationsPerSecond(0),
          indexer);
    }
  }

  public static Command getOuttakeCommand(Indexer indexer) {
    return new StartEndCommand(
        () -> indexer.setVelocityRotationsPerSecond(-3),
        () -> indexer.setVelocityRotationsPerSecond(0),
        indexer);
  }

  public static Command getSetFeederCommand(DoubleSupplier velocity, Feeder feeder) {
    if (velocity.getAsDouble() == 0) {
      return new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder);
    } else {
      return new StartEndCommand(
          () -> feeder.setVelocityRotationsPerSecond(velocity.getAsDouble()),
          () -> feeder.setVelocityRotationsPerSecond(0),
          feeder);
    }
  }

  public static Command getSetShooterOnlyCommand(DoubleSupplier velocity, Shooter shooter) {
    if (velocity.getAsDouble() == 0) {
      return new InstantCommand(() -> shooter.setShooterVelocityRotationsPerSecond(0), shooter);
    } else {
      return new StartEndCommand(
          () -> shooter.setShooterVelocityRotationsPerSecond(velocity.getAsDouble()),
          () -> shooter.setShooterVelocityRotationsPerSecond(0),
          shooter);
    }
  }

  public static Command getSetShooterCommand(
      DoubleSupplier shooterVelocity, DoubleSupplier tangentialVelocityRatio, Shooter shooter) {
    return new StartEndCommand(
        () ->
            shooter.setVelocities(
                shooterVelocity.getAsDouble(), tangentialVelocityRatio.getAsDouble()),
        () -> shooter.setVelocities(0, 1),
        shooter);
  }

  public static Command getSetIntakeCommand(DoubleSupplier velocity, Intake intake) {
    if (velocity.getAsDouble() == 0) {
      return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0), intake);
    } else {
      return new StartEndCommand(
          () -> intake.setVelocityRotationsPerSecond(velocity.getAsDouble()),
          () -> intake.setVelocityRotationsPerSecond(0),
          intake);
    }
  }

  public static Command getSetServoCommand(BooleanSupplier engaged, FeedServo servo) {
    return new InstantCommand(() -> servo.engage(engaged.getAsBoolean()), servo);
  }

  public static Command getIntakeArmUpCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPosition(true), intakeArm)
        .withInterrupt(() -> intakeArm.atPosition(true));
  }

  public static Command getIntakeArmDownCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPosition(false), intakeArm)
        .withInterrupt(() -> intakeArm.atPosition(false));
  }
}
