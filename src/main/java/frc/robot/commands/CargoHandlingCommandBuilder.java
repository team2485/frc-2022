package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  public static Command getIndexerSetCommand(Indexer indexer, FeedServo servo) {
    return new RunCommand(
            () ->
                indexer.setVelocityRotationsPerSecond(
                    kIndexerIntakeSpeedRatio * kIntakeDefaultSpeedRotationsPerSecond),
            indexer)
        .alongWith(new InstantCommand(() -> servo.engage(false), servo));
  }

  public static Command getStopIntakeCommand(Intake intake, IntakeArm intakeArm, Indexer indexer) {
    return getIntakeArmUpCommand(intakeArm)
        .alongWith(
            new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer),
            new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0), intake));
  }

  public static Command getTurretAutoAimCommand(
      Turret turret, Supplier<Pose2d> robotPose, DoubleSupplier turretShift) {
    return new RunCommand(
        () -> turret.setAngleRadians(findTurretAimAngle(robotPose) + turretShift.getAsDouble()),
        turret);
  }

  public static Command getTurretSetCommand(Turret turret, DoubleSupplier angle) {
    return new RunCommand(() -> turret.setAngleRadians(angle.getAsDouble()), turret);
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

  public static Command getHoodAutoAimCommand(
      Hood hood, DoubleSupplier distanceToHub, DoubleSupplier distanceOffset) {
    return getSetHoodCommand(getHoodAutoSetpoint(distanceToHub, distanceOffset), hood);
  }

  public static DoubleSupplier getHoodAutoSetpoint(
      DoubleSupplier distanceToHub, DoubleSupplier distanceOffset) {
    return () ->
        InterpolatingTable.get(distanceToHub.getAsDouble() + distanceOffset.getAsDouble())
            .hoodAngleRadians;
  }

  public static Command getShooterAutoSetCommand(
      Shooter shooter, DoubleSupplier distanceToHub, DoubleSupplier distanceOffset) {
    return getShooterSetCommand(shooter, getShooterAutoSetpoint(distanceToHub, distanceOffset));
  }

  public static Command getShooterSetCommand(Shooter shooter, DoubleSupplier velocity) {
    return new RunCommand(
        () -> shooter.setVelocityRotationsPerSecond(velocity.getAsDouble()), shooter);
  }

  public static DoubleSupplier getShooterAutoSetpoint(
      DoubleSupplier distanceToHub, DoubleSupplier distanceOffset) {
    return () ->
        InterpolatingTable.get(distanceToHub.getAsDouble() + distanceOffset.getAsDouble())
            .shooterSpeedRotationsPerSecond;
  }

  public static Command getEjectCommand(
      Shooter shooter,
      Hood hood,
      Turret turret,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Supplier<Pose2d> robotPose) {
    return new RunCommand(() -> shooter.setVelocityRotationsPerSecond(40), shooter)
        .alongWith(
            new RunCommand(() -> hood.setAngleRadians(kHoodBottomPositionRadians), hood),
            new RunCommand(() -> turret.setAngleRadians(findTurretEjectAngle(robotPose)), turret),
            new WaitCommand(1)
                .andThen(getIndexToShooterOnceCommand(indexer, feeder, servo, shooter)));
  }

  public static double findTurretEjectAngle(Supplier<Pose2d> robotPose) {
    double angle = findTurretAimAngle(robotPose);
    if (angle > 0) {
      angle = angle - kTurretRangeRadians / 4.0;
    } else {
      angle = angle + kTurretRangeRadians / 4.0;
    }
    return angle;
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
    return new WaitUntilCommand(
            () -> {
              if (shooter.getSetpoint() > 80) {
                return shooter.withinTolerance(kShooterFeedVelocityToleranceHigh);
              } else {
                return true;
              }
            })
        .andThen(
            new ParallelRaceGroup(
                getSetFeederCommand(() -> kFeederDefaultSpeedRotationsPerSecond, feeder),
                new WaitCommand(0.2)
                    .andThen(getSetServoCommand(() -> true, servo), new WaitCommand(0.5))),
            getSetFeederCommand(() -> 0, feeder).alongWith(getSetServoCommand(() -> false, servo)),
            new WaitCommand(0.3),
            getSetIndexerCommand(() -> kIndexerDefaultSpeedRotationsPerSecond, indexer)
                .withTimeout(0.5));
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

  public static Command getSetShooterCommand(DoubleSupplier velocity, Shooter shooter) {
    if (velocity.getAsDouble() == 0) {
      return new InstantCommand(() -> shooter.setVelocityRotationsPerSecond(0), shooter);
    } else {
      return new StartEndCommand(
          () -> shooter.setVelocityRotationsPerSecond(velocity.getAsDouble()),
          () -> shooter.setVelocityRotationsPerSecond(0),
          shooter);
    }
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

  public static Command getSetHoodCommand(DoubleSupplier angle, Hood hood) {
    return new RunCommand(() -> hood.setAngleRadians(angle.getAsDouble()), hood)
        .until(hood::atGoal);
  }

  public static Command getHoodDownCommand(Hood hood) {
    return new InstantCommand(() -> hood.setAngleRadians(kHoodBottomPositionRadians));
  }
}
