package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;

public class AutoCommandBuilder {
  public static Command get2BallAuto(
      Drivetrain drivetrain,
      Vision vision,
      Intake intake,
      IntakeArm intakeArm,
      Indexer lowIndexer,
      Feeder highIndexer,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand backUpPath =
        PathCommandBuilder.getPathCommand(drivetrain, "2 ball right");

    InstantCommand resetOdometry =
        new InstantCommand(
            () -> {
              drivetrain.resetOdometry(backUpPath.m_trajectory.getInitialPose(), false);
            });

    Command intakeBalls =
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, lowIndexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter,
            drivetrain::getHubToTurretCenterDistanceMeters,
            drivetrain::getFieldRelativeVelocityMetersPerSecond);
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(
            lowIndexer, highIndexer, servo, shooter);

    Command turnLEDsOn = new InstantCommand(() -> vision.setForceLeds(true));

    return turnLEDsOn
        .alongWith(resetOdometry)
        .andThen(
            backUpPath.alongWith(intakeBalls, setShooter), feedToShooterOnce, feedToShooterOnce);
  }

  public static Command get3BallAuto(
      Drivetrain drivetrain,
      Vision vision,
      Intake intake,
      IntakeArm intakeArm,
      Indexer lowIndexer,
      Feeder highIndexer,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand thirdBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "2 to 3 ball right");

    Command intakeBalls =
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, lowIndexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter,
            drivetrain::getHubToTurretCenterDistanceMeters,
            drivetrain::getFieldRelativeVelocityMetersPerSecond);
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(
            lowIndexer, highIndexer, servo, shooter);

    return get2BallAuto(
            drivetrain, vision, intake, intakeArm, lowIndexer, highIndexer, servo, shooter)
        .andThen(thirdBallPath.alongWith(intakeBalls, setShooter), feedToShooterOnce);
  }

  //       Drivetrain drivetrain,
  //       Vision vision,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer lowIndexer,
  //       Feeder highIndexer,
  //       Shooter shooter,
  //       BallCounter counter) {

  //     WL_SwerveControllerCommand thirdAndFourthBallPath =
  //         PathCommandBuilder.getPathCommand(drivetrain, "2 to 4 ball right");

  //     Command intakeBalls =
  //         CargoHandlingCommandBuilder.getIntakeCommand(
  //             intake, intakeArm, lowIndexer, highIndexer, counter);
  //     Command setShooter =
  //         CargoHandlingCommandBuilder.getShooterAutoSetCommand(
  //             shooter,
  //             drivetrain::getDistanceToHubMeters,
  //             drivetrain::getFieldRelativeVelocityMetersPerSecond);
  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(lowIndexer, highIndexer,
  // shooter);

  //     return get2BallAuto(
  //             drivetrain, vision, intake, intakeArm, lowIndexer, highIndexer, shooter, counter)
  //         .andThen(
  //             thirdAndFourthBallPath.alongWith(intakeBalls, setShooter),
  //             feedToShooterOnce,
  //             feedToShooterOnce);
  //   }

  //   public static Command get5BallAuto(
  //       Drivetrain drivetrain,
  //       Vision vision,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer lowIndexer,
  //       Feeder highIndexer,
  //       Shooter shooter,
  //       BallCounter counter) {

  //     WL_SwerveControllerCommand thirdAndFourthBallPath =
  //         PathCommandBuilder.getPathCommand(drivetrain, "2 to 4 ball right");

  //     Command intakeBalls =
  //         CargoHandlingCommandBuilder.getIntakeCommand(
  //             intake, intakeArm, lowIndexer, highIndexer, counter);
  //     Command setShooter =
  //         CargoHandlingCommandBuilder.getShooterAutoSetCommand(
  //             shooter,
  //             drivetrain::getDistanceToHubMeters,
  //             drivetrain::getFieldRelativeVelocityMetersPerSecond);
  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(lowIndexer, highIndexer,
  // shooter);

  //     return get2BallAuto(
  //             drivetrain, vision, intake, intakeArm, lowIndexer, highIndexer, shooter, counter)
  //         .andThen(
  //             thirdAndFourthBallPath.alongWith(intakeBalls, setShooter),
  //             feedToShooterOnce,
  //             feedToShooterOnce,
  //             intakeBalls.alongWith(setShooter),
  //             feedToShooterOnce);
  //   }

  public static Command setLEDsAutoCommand(Vision vision) {
    return new InstantCommand(() -> vision.setForceLeds(false));
  }
}
