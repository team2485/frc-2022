package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
import frc.robot.subsystems.cargoHandling.FeedServo;
import frc.robot.subsystems.cargoHandling.Feeder;
import frc.robot.subsystems.cargoHandling.Hood;
import frc.robot.subsystems.cargoHandling.Indexer;
import frc.robot.subsystems.cargoHandling.Intake;
import frc.robot.subsystems.cargoHandling.IntakeArm;
import frc.robot.subsystems.cargoHandling.Shooter;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CargoHandlingCommandBuilder {

  public static Command allignToHub(Drivetrain drivetrain) {

    // kP * tx
    return new InstantCommand(
        () ->
            drivetrain.drive(
                0,
                0,
                (Math.abs(
                                NetworkTableInstance.getDefault()
                                    .getTable("limelight")
                                    .getEntry("tx")
                                    .getDouble(0))
                            > 1
                        ? -0.1
                        : 0)
                    * NetworkTableInstance.getDefault()
                        .getTable("limelight")
                        .getEntry("tx")
                        .getDouble(0),
                false));
  }

  public static Command setShooterForShot(Hood hood, Shooter shooter) {
    return new InstantCommand(() -> shooter.allignToHub())
        .andThen(new InstantCommand(() -> hood.allignToHub()));
  }

  public static Command getRunFeederCommand(Feeder feeder, Indexer indexer) {
    return new RunCommand(() -> feeder.setVelocityRotationsPerSecond(4))
        .alongWith(new RunCommand(() -> indexer.setVelocityRotationsPerSecond(6)));
  }

  public static Command getStopFeederCommand(Feeder feeder, Indexer indexer) {
    return new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder)
        .alongWith(new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0)));
  }

  public static Command getRunIndexerCommand(Indexer indexer) {
    return new RunCommand(() -> indexer.setVelocityRotationsPerSecond(6), indexer)
        .withTimeout(0.5)
        .andThen(new RunCommand(() -> indexer.setVelocityRotationsPerSecond(6)));
  }

  public static Command getStopIndexerCommand(Indexer indexer) {
    return new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer);
  }

  public static Command runTestCommand(Intake intake, IntakeArm intakeArm, Indexer indexer) {
    return getArmDownCommand(intakeArm)
        .andThen(
            new RunCommand(
                () -> intake.setVelocityRotationsPerSecond(kIntakeDefaultSpeedRotationsPerSecond)))
        .alongWith(new RunCommand(() -> indexer.setVelocityRotationsPerSecond(6)));
  }

  // no intake arm
  public static Command intakeForAutoCommand(Intake intake, Indexer indexer) {
    return new RunCommand(
            () -> intake.setVelocityRotationsPerSecond(kIntakeDefaultSpeedRotationsPerSecond))
        .alongWith(new RunCommand(() -> indexer.setVelocityRotationsPerSecond(6)));
  }

  public static Command stopIntakeForAutoCommand(Intake intake, Indexer indexer) {
    return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0))
        .alongWith(new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0)));
  }

  public static Command outtakeCommand(Intake intake, IntakeArm intakeArm) {
    return getArmDownCommand(intakeArm)
        .andThen(new RunCommand(() -> intake.setVelocityRotationsPerSecond(-6)));
  }

  public static Command stopTestCommand(Intake intake, IntakeArm intakeArm, Indexer indexer) {
    return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0))
        .andThen(
            new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0)),
            getArmUpCommand(intakeArm));
  }

  public static Command getIntakeCommand(
      Intake intake, IntakeArm intakeArm, Indexer indexer, FeedServo servo) {
    return getArmUpCommand(intakeArm)
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
    return getArmDownCommand(intakeArm)
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
      return new InstantCommand(() -> shooter.zeroShooter(), shooter);
    } else {
      return new StartEndCommand(
          () -> shooter.setShooterVelocityRotationsPerSecond(),
          () -> shooter.zeroShooter(),
          shooter);
    }
  }

  public static Command getSetShooterCommand(Shooter shooter) {
    return new StartEndCommand(() -> shooter.setVelocities(), () -> shooter.zeroShooter(), shooter);
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

  public static Command getArmUpCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.setArmUp(), intakeArm);
  }

  public static Command getArmDownCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm);
  }

  public static Command toggleArmCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.togglePosition(), intakeArm);
  }
}
