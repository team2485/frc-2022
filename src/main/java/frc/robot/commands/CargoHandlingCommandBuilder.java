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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
import frc.robot.subsystems.cargoHandling.BallCounter;
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
      Intake intake, IntakeArm intakeArm, Indexer indexer, BallCounter counter) {
    return getIntakeArmDownCommand(intakeArm)
        .andThen(
            new RunCommand(
                () -> intake.setVelocityRotationsPerSecond(kIntakeDefaultSpeedRotationsPerSecond),
                intake))
        .alongWith(
            new RunCommand(
                () -> indexer.setVelocityRotationsPerSecond(kIndexerDefaultSpeedRotationsPerSecond),
                indexer))
        .until(
            () -> {
              return counter.getNumCargo() == 2 || indexer.hasStopped();
            });
  }

  public static Command getIntakeArmUpCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setAngleRadians(kIntakeArmTopPositionRadians), intakeArm)
        .withInterrupt(intakeArm::atGoal);
  }

  public static Command getIntakeArmDownCommand(IntakeArm intakeArm) {
    return new RunCommand(
            () -> intakeArm.setAngleRadians(kIntakeArmBottomPositionRadians), intakeArm)
        .withInterrupt(intakeArm::atGoal);
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

  public static Command getIndexToShooterCommand(
      Indexer indexer, Feeder feeder, Shooter shooter, BallCounter counter) {
    return new ConditionalCommand(
        getIndexToShooterOnceCommand(indexer, feeder, shooter)
            .andThen(
                new RunCommand(
                        () ->
                            indexer.setVelocityRotationsPerSecond(
                                kIndexerDefaultSpeedRotationsPerSecond),
                        indexer)
                    .withTimeout(0.5)),
        new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer)
            .alongWith(new InstantCommand(() -> shooter.setVelocityRotationsPerSecond(0))),
        () -> {
          return counter.getNumCargo() > 0;
        });
  }

  public static Command getIndexToShooterOnceCommand(
      Indexer indexer, Feeder feeder, Shooter shooter) {
    return new WaitUntilCommand(shooter::atSetpoint)
        .andThen(
            new RunCommand(
                    () ->
                        feeder.setVelocityRotationsPerSecond(
                            shooter.getVelocityRotationsPerSecond()
                                * kShooterCircumferenceMeters
                                * kFeederShooterSurfaceSpeedRatio
                                / kFeederPulleyCircumferenceMeters),
                    feeder)
                .alongWith(
                    new WaitCommand(0.1)
                        .andThen(new InstantCommand(() -> feeder.engageServo(true), feeder)))
                .until(shooter::hasDipped)
                .andThen(
                    getFeederOffCommand(feeder)
                        .alongWith(new InstantCommand(() -> feeder.engageServo(false), feeder))));
  }

  public static Command getZeroHoodCommand(Hood hood) {
    return new ConditionalCommand(
        new InstantCommand(() -> hood.setVoltage(kHoodZeroingVoltage), hood),
        new InstantCommand(() -> hood.setVoltage(0), hood),
        hood::getBottomLimitSwitch);
  }

  public static Command getShooterOffCommand(Shooter shooter) {
    return new InstantCommand(() -> shooter.setVelocityRotationsPerSecond(0), shooter);
  }

  public static Command getIndexerOffCommand(Indexer indexer) {
    return new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0), indexer);
  }

  public static Command getFeederOffCommand(Feeder feeder) {
    return new InstantCommand(() -> feeder.setVelocityRotationsPerSecond(0), feeder);
  }

  public static Command getIntakeOffCommand(Intake intake) {
    return new InstantCommand(() -> intake.setVelocityRotationsPerSecond(0), intake);
  }

  public static Command getIntakeArmOffCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.setVoltage(0), intakeArm);
  }
}
