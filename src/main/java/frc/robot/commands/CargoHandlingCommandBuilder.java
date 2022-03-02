package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
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

  public static Command getIntakeCommand(Intake intake, IntakeArm intakeArm, Indexer indexer) {
    return getIntakeArmDownCommand(intakeArm)
        .andThen(new RunCommand(() -> intake.setPercentOutput(kIntakePercentOutputIn), intake))
        .alongWith(
            new RunCommand(() -> indexer.setPercentOutput(kIndexerPercentOutputIn), indexer)
                .withInterrupt(indexer::hasStopped));
  }

  public static Command getIntakeArmUpCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPercentOutput(kIntakeArmPercentOutputUp), intakeArm)
        .withInterrupt(intakeArm::getTopLimitSwitch);
  }

  public static Command getIntakeArmDownCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPercentOutput(kIntakeArmPercentOutputDown), intakeArm)
        .withInterrupt(intakeArm::getBottomLimitSwitch);
  }

  public static Command getTurretAutoAimCommand(
      Turret turret, Supplier<Pose2d> robotPose, Supplier<Translation2d> robotVelocity) {
    return new RunCommand(
        () ->
            turret.setAngleRadians(
                Math.atan(
                        (robotPose.get().getX() - kHubCenterPosition.getX())
                            / (robotPose.get().getX() - kHubCenterPosition.getY()))
                    - robotPose.get().getRotation().getRadians()),
        turret);
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

  public static Command getIndexToShooterCommand(Indexer indexer, Feeder feeder, Shooter shooter) {
    return new ConditionalCommand(
        new InstantCommand(
            () -> {
              indexer.setPercentOutput(kIndexerPercentOutputFeedToShooter);
              feeder.setPercentOutput(kFeederPercentOutputFeedToShooter);
            },
            indexer,
            feeder),
        new InstantCommand(
            () -> {
              indexer.setPercentOutput(0);
              feeder.setPercentOutput(0);
            },
            indexer,
            feeder),
        shooter::atSetpoint);
  }

  public static Command getIndexToShooterOnceCommand(
      Indexer indexer, Feeder feeder, Shooter shooter) {

    return getIndexToShooterCommand(indexer, feeder, shooter)
        .withInterrupt(shooter::hasDipped)
        .andThen(
            new InstantCommand(
                () -> {
                  indexer.setPercentOutput(0);
                  feeder.setPercentOutput(0);
                },
                indexer,
                feeder));
  }

  public static Command getZeroHoodCommand(Hood hood) {
    return new ConditionalCommand(
        new InstantCommand(() -> hood.setPercentOutput(kHoodZeroingPercentOutput), hood),
        new InstantCommand(() -> hood.setPercentOutput(0), hood),
        hood::getBottomLimitSwitch);
  }

  public static Command getShooterOffCommand(Shooter shooter) {
    return new InstantCommand(() -> shooter.setVelocityRotationsPerSecond(0), shooter);
  }

  public static Command getIndexerOffCommand(Indexer indexer) {
    return new InstantCommand(() -> indexer.setPercentOutput(0), indexer);
  }

  public static Command getFeederOffCommand(Feeder feeder) {
    return new InstantCommand(() -> feeder.setPercentOutput(0), feeder);
  }

  public static Command getIntakeOffCommand(Intake intake) {
    return new InstantCommand(() -> intake.setPercentOutput(0), intake);
  }

  public static Command getIntakeArmOffCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.setPercentOutput(0), intakeArm);
  }
}
