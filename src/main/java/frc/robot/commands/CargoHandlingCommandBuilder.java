package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
import frc.robot.subsystems.cargoHandling.FeedServo;
import frc.robot.subsystems.cargoHandling.Feeder;
import frc.robot.subsystems.cargoHandling.Hood;
import frc.robot.subsystems.cargoHandling.Indexer;
import frc.robot.subsystems.cargoHandling.Intake;
import frc.robot.subsystems.cargoHandling.IntakeArm;
import frc.robot.subsystems.cargoHandling.Shooter;
import frc.robot.subsystems.cargoHandling.Turret;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CargoHandlingCommandBuilder {

  public CargoHandlingCommandBuilder() {}

  public static Command getIntakeCommand(
      Intake intake, IntakeArm intakeArm, Indexer indexer, FeedServo servo) {
    return getIntakeArmDownCommand(intakeArm)
        .andThen(
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

  // .raceWith(
  //     new WaitUntilCommand(
  //             () -> {
  //               return counter.getNumCargo() == 2;
  //             })
  //         .andThen(new WaitCommand(1)));

  public static Command getIntakeArmUpCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPosition(true), intakeArm)
        .withInterrupt(() -> intakeArm.atPosition(true));
  }

  public static Command getIntakeArmDownCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPosition(false), intakeArm)
        .withInterrupt(() -> intakeArm.atPosition(false));
  }

  public static Command getTurretAutoAimCommand(
      Turret turret, Supplier<Pose2d> robotPose, Supplier<Translation2d> robotVelocity) {
    return new RunCommand(() -> turret.setAngleRadians(findTurretAimAngle(robotPose)), turret);
  }

  public static double findTurretAimAngle(Supplier<Pose2d> robotPose) {
    double uncorrectedAngle =
        Math.atan(
                (kHubCenterPosition.getY() - robotPose.get().getY())
                    / (kHubCenterPosition.getX() - robotPose.get().getX()))
            - robotPose.get().getRotation().getRadians();
    if (robotPose.get().getX() <= kHubCenterPosition.getX()) {
      return uncorrectedAngle;
    } else {
      return uncorrectedAngle + Math.PI;
    }
  }

  public static Command getHoodShooterAutoAimCommand(
      Hood hood,
      Shooter shooter,
      DoubleSupplier distanceToHub,
      Supplier<Translation2d> robotVelocity) {
    return getHoodAutoAimCommand(hood, distanceToHub, robotVelocity)
        .alongWith(getShooterAutoSetCommand(shooter, distanceToHub, robotVelocity));
  }

  public static Command getHoodAutoAimCommand(
      Hood hood, DoubleSupplier distanceToHub, Supplier<Translation2d> robotVelocity) {
    return new RunCommand(
        () ->
            hood.setAngleRadians(
                InterpolatingTable.get(distanceToHub.getAsDouble()).hoodAngleRadians),
        hood);
  }

  public static Command getShooterAutoSetCommand(
      Shooter shooter, DoubleSupplier distanceToHub, Supplier<Translation2d> robotVelocity) {
    return new RunCommand(
        () ->
            shooter.setVelocityRotationsPerSecond(
                InterpolatingTable.get(distanceToHub.getAsDouble()).shooterSpeedRotationsPerSecond),
        shooter);
  }

  // public static Command getIndexToShooterCommand(
  //     Indexer indexer, Feeder feeder, Shooter shooter, BallCounter counter) {
  //   return new ConditionalCommand(
  //       getIndexToShooterOnceCommand(indexer, feeder, shooter)
  //           .andThen(
  //               new RunCommand(
  //                       () ->
  //                           indexer.setVelocityRotationsPerSecond(
  //                               kIndexerDefaultSpeedRotationsPerSecond),
  //                       indexer)
  //                   .withTimeout(0.5)),
  //       new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer)
  //           .alongWith(new InstantCommand(() -> shooter.setVelocityRotationsPerSecond(0))),
  //       () -> {
  //         return counter.getNumCargo() > 0;
  //       });
  // }

  public static Command getIndexToShooterOnceCommand(
      Indexer indexer, Feeder feeder, FeedServo servo, Shooter shooter) {
    return new WaitUntilCommand(() -> shooter.withinTolerance(kShooterFeedVelocityTolerance))
        .andThen(
            new ParallelRaceGroup(
                new RunCommand(
                    () ->
                        feeder.setVelocityRotationsPerSecond(kFeederDefaultSpeedRotationsPerSecond),
                    feeder),
                new WaitCommand(0.2)
                    .andThen(
                        new InstantCommand(() -> servo.engage(true), servo), new WaitCommand(1))),
            new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder)
                .alongWith(new InstantCommand(() -> servo.engage(false), servo)),
            new WaitCommand(0.5),
            new InstantCommand(
                () -> indexer.setVelocityRotationsPerSecond(kIndexerDefaultSpeedRotationsPerSecond),
                indexer),
            new WaitCommand(0.5),
            new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer));
  }

  public static Command getStopFeedCommand(Indexer indexer, Feeder feeder, FeedServo servo) {
    return new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer)
        .alongWith(
            new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder),
            new InstantCommand(() -> servo.engage(false), servo));
  }

  public static Command getZeroHoodCommand(Hood hood) {
    return new ConditionalCommand(
        new InstantCommand(() -> hood.setVoltage(kHoodZeroingVoltage), hood),
        new InstantCommand(() -> hood.setVoltage(0), hood),
        hood::getBottomLimitSwitch);
  }

  public static Command getHoodDownCommand(Hood hood) {
    return new InstantCommand(() -> hood.setAngleRadians(kHoodBottomPositionRadians));
  }

  public static Command getShooterOffCommand(Shooter shooter) {
    return new RunCommand(() -> shooter.setVelocityRotationsPerSecond(0), shooter);
  }

  public static Command getIndexerOffCommand(Indexer indexer) {
    return new RunCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer);
  }

  public static Command getFeederOffCommand(Feeder feeder) {
    return new RunCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder);
  }

  public static Command getIntakeOffCommand(Intake intake) {
    return new RunCommand(() -> intake.setVelocityRotationsPerSecond(0), intake);
  }

  public static Command getIntakeArmOffCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setVoltage(0), intakeArm);
  }
}
