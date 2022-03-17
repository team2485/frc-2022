package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      Indexer indexer,
      Feeder feeder,
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
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter,
            drivetrain::getHubToTurretCenterDistanceMeters,
            () -> {
              return 0;
            });
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(indexer, feeder, servo, shooter);

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
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand thirdBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "2 to 3 ball right");

    Command intakeBalls =
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter,
            drivetrain::getHubToTurretCenterDistanceMeters,
            () -> {
              return 0;
            });
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(indexer, feeder, servo, shooter);

    return get2BallAuto(drivetrain, vision, intake, intakeArm, indexer, feeder, servo, shooter)
        .andThen(thirdBallPath.alongWith(intakeBalls, setShooter), feedToShooterOnce);
  }

  public static Command get4BallAuto(
      Drivetrain drivetrain,
      Vision vision,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand thirdAndFourthBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "2 to 4 ball right");

    WL_SwerveControllerCommand scoreBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "4/5 to score right");

    Command intakeBalls =
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter, drivetrain::getHubToTurretCenterDistanceMeters, () -> 0);
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(indexer, feeder, servo, shooter);

    return get2BallAuto(drivetrain, vision, intake, intakeArm, indexer, feeder, servo, shooter)
        .andThen(
            thirdAndFourthBallPath.alongWith(intakeBalls, setShooter),
            new WaitCommand(1),
            scoreBallPath.alongWith(setShooter),
            feedToShooterOnce,
            feedToShooterOnce);
  }

  public static Command get5BallAuto(
      Drivetrain drivetrain,
      Vision vision,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand fourthAndFifthBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "3 to 5 ball right");

    WL_SwerveControllerCommand scoreBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "4/5 to score right");

    Command intakeBalls =
        CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
    Command setShooter =
        CargoHandlingCommandBuilder.getShooterAutoSetCommand(
            shooter, drivetrain::getHubToTurretCenterDistanceMeters, () -> 0);
    Command feedToShooterOnce =
        CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(indexer, feeder, servo, shooter);

    return get3BallAuto(drivetrain, vision, intake, intakeArm, indexer, feeder, servo, shooter)
        .andThen(
            fourthAndFifthBallPath.alongWith(intakeBalls, setShooter),
            new WaitCommand(3),
            scoreBallPath.alongWith(setShooter),
            feedToShooterOnce,
            feedToShooterOnce);
  }

  public static Command setLEDsAutoCommand(Vision vision) {
    return new InstantCommand(() -> vision.setForceLeds(false));
  }
}
