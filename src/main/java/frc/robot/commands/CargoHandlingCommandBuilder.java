package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.interpolation.InterpolatingTable;
import frc.robot.subsystems.cargoHandling.Hood;
import frc.robot.subsystems.cargoHandling.Intake;
import frc.robot.subsystems.cargoHandling.IntakeArm;
import frc.robot.subsystems.cargoHandling.Shooter;
import frc.robot.subsystems.cargoHandling.Turret;
import frc.robot.subsystems.cargoHandling.indexing.HighIndexer;
import frc.robot.subsystems.cargoHandling.indexing.LowIndexer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CargoHandlingCommandBuilder {

  public CargoHandlingCommandBuilder() {}

  public static Command getIntakeCommand(
      Intake intake, IntakeArm intakeArm, LowIndexer lowIndexer) {
    return getIntakeArmDownCommand(intakeArm)
        .andThen(new RunCommand(() -> intake.setPercentOutput(kIntakePercentOutputIn), intake))
        .alongWith(
            new RunCommand(
                    () -> lowIndexer.setPercentOutput(kLowIndexerPercentOutputIn), lowIndexer)
                .withInterrupt(lowIndexer::hasStopped));
  }

  public static Command getIntakeArmUpCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPercentOutput(kIntakeArmPercentOutputUp), intakeArm)
        .withInterrupt(intakeArm::getTopLimitSwitch);
  }

  public static Command getIntakeArmDownCommand(IntakeArm intakeArm) {
    return new RunCommand(() -> intakeArm.setPercentOutput(kIntakeArmPercentOutputDown), intakeArm)
        .withInterrupt(intakeArm::getBottomLimitSwitch);
  }

  public static Command getTurretAutoAimCommand(Turret turret, Supplier<Pose2d> robotPose) {
    return new RunCommand(
        () ->
            turret.setAngleRadians(
                Math.atan(
                        (robotPose.get().getX() - kHubCenterPosition.getX())
                            / (robotPose.get().getX() - kHubCenterPosition.getY()))
                    - robotPose.get().getRotation().getRadians()),
        turret);
  }

  public static Command getHoodAutoAimCommand(Hood hood, DoubleSupplier distanceToHub) {
    return new RunCommand(
        () ->
            hood.setAngleRadians(
                InterpolatingTable.get(distanceToHub.getAsDouble()).hoodAngleRadians),
        hood);
  }

  public static Command getShooterAutoSetCommand(Shooter shooter, DoubleSupplier distanceToHub) {
    return new RunCommand(
        () ->
            shooter.setVelocityRotationsPerSecond(
                InterpolatingTable.get(distanceToHub.getAsDouble()).shooterSpeedRotationsPerSecond),
        shooter);
  }

  public static Command getIndexToShooterCommand(
      LowIndexer lowIndexer, HighIndexer highIndexer, Shooter shooter) {
    return new ConditionalCommand(
        new InstantCommand(
            () -> {
              lowIndexer.setPercentOutput(kLowIndexerPercentOutputFeedToShooter);
              highIndexer.setPercentOutput(kHighIndexerPercentOutputFeedToShooter);
            },
            lowIndexer,
            highIndexer),
        new InstantCommand(
            () -> {
              lowIndexer.setPercentOutput(0);
              highIndexer.setPercentOutput(0);
            },
            lowIndexer,
            highIndexer),
        shooter::atSetpoint);
  }

  public static Command getIndexToShooterOnceCommand(
      LowIndexer lowIndexer, HighIndexer highIndexer, Shooter shooter) {

    return getIndexToShooterCommand(lowIndexer, highIndexer, shooter)
        .withInterrupt(shooter::hasDipped)
        .andThen(
            new InstantCommand(
                () -> {
                  lowIndexer.setPercentOutput(0);
                  highIndexer.setPercentOutput(0);
                },
                lowIndexer,
                highIndexer));
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

  public static Command getLowIndexerOffCommand(LowIndexer lowIndexer) {
    return new InstantCommand(() -> lowIndexer.setPercentOutput(0), lowIndexer);
  }

  public static Command getHighIndexerOffCommand(HighIndexer highIndexer) {
    return new InstantCommand(() -> highIndexer.setPercentOutput(0), highIndexer);
  }

  public static Command getIntakeOffCommand(Intake intake) {
    return new InstantCommand(() -> intake.setPercentOutput(0), intake);
  }

  public static Command getIntakeArmOffCommand(IntakeArm intakeArm) {
    return new InstantCommand(() -> intakeArm.setPercentOutput(0), intakeArm);
  }
}
