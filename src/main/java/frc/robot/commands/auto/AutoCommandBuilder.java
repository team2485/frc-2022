package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;

public class AutoCommandBuilder {

  public static Command getFinishAutoCommand(
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {
    return CargoHandlingCommandBuilder.getIntakeOffCommand(intake)
        .alongWith(
            CargoHandlingCommandBuilder.getIntakeArmUpCommand(intakeArm),
            CargoHandlingCommandBuilder.getIndexerOffCommand(indexer),
            CargoHandlingCommandBuilder.getFeederOffCommand(feeder),
            CargoHandlingCommandBuilder.getShooterOffCommand(shooter),
            CargoHandlingCommandBuilder.getHoodDownCommand(hood),
            CargoHandlingCommandBuilder.getFeedServoOpenCommand(servo))
        .withTimeout(0.5);
  }

  public static Command getIntakeBallsCommand(
      Intake intake, IntakeArm intakeArm, Indexer indexer, FeedServo servo) {
    return CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
  }

  public static Command getSetShooterCommand(Shooter shooter, Drivetrain drivetrain) {
    return CargoHandlingCommandBuilder.getShooterAutoSetCommand(
        shooter, drivetrain::getHubToTurretCenterDistanceMeters, () -> 0);
  }

  public static Command getFeedToShooterCommand(
      Hood hood,
      Drivetrain drivetrain,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter) {
    return CargoHandlingCommandBuilder.getHoodAutoAimCommand(
            hood, drivetrain::getHubToTurretCenterDistanceMeters, () -> 0)
        .alongWith(
            CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(
                indexer, feeder, servo, shooter));
  }

  public static Command get2BallAutoRight(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    Command backUpPath = PathCommandBuilder.getPathCommand(drivetrain, "2 ball right");

    Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);

    Command setShooter = getSetShooterCommand(shooter, drivetrain);

    Command feedToShooterOnce =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    Command feedToShooterOnce2 =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    return CargoHandlingCommandBuilder.getFeedServoOpenCommand(servo)
        .andThen(
            new ParallelRaceGroup(backUpPath.withTimeout(1), intakeBalls.alongWith(setShooter)),
            feedToShooterOnce,
            feedToShooterOnce2);
  }

  public static Command get2BallAutoLeft(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    Command backUpPath = PathCommandBuilder.getPathCommand(drivetrain, "2 ball left");

    Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);

    Command setShooter = getSetShooterCommand(shooter, drivetrain);

    Command feedToShooterOnce =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    Command feedToShooterOnce2 =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    return new ParallelRaceGroup(
            backUpPath.andThen(new WaitCommand(1)), intakeBalls.alongWith(setShooter))
        .andThen(feedToShooterOnce, feedToShooterOnce2);
  }

  public static Command get3BallAuto(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    Command thirdBallPath = PathCommandBuilder.getPathCommand(drivetrain, "2 to 3 ball right");

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

    return get2BallAutoRight(drivetrain, intake, intakeArm, indexer, feeder, servo, shooter, hood)
        .andThen(thirdBallPath.alongWith(intakeBalls, setShooter), feedToShooterOnce);
  }

  public static Command get4BallAuto(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    Command thirdAndFourthBallPath =
        PathCommandBuilder.getPathCommand(drivetrain, "2 to 4 ball right");

    Command scoreBallPath = PathCommandBuilder.getPathCommand(drivetrain, "4 or 5 to score right");

    Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);

    Command setShooter1 = getSetShooterCommand(shooter, drivetrain);
    Command setShooter2 = getSetShooterCommand(shooter, drivetrain);

    Command feedToShooterOnce =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    Command feedToShooterOnce2 =
        getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

    return get2BallAutoRight(drivetrain, intake, intakeArm, indexer, feeder, servo, shooter, hood)
        .andThen(
            new ParallelRaceGroup(
                thirdAndFourthBallPath.andThen(new WaitCommand(0.5)),
                intakeBalls.alongWith(setShooter1)),
            new ParallelRaceGroup(scoreBallPath.andThen(new WaitCommand(0.2)), setShooter2),
            feedToShooterOnce,
            feedToShooterOnce2);
  }

  //   public static Command get5BallAuto(
  //       Drivetrain drivetrain,
  //       Vision vision,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter) {

  //     Command fourthAndFifthBallPath =
  //         PathCommandBuilder.getPathCommand(drivetrain, "3 to 5 ball right");

  //     Command scoreBallPath = PathCommandBuilder.getPathCommand(drivetrain, "4/5 to score
  // right");

  //     Command intakeBalls =
  //         CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
  //     Command setShooter =
  //         CargoHandlingCommandBuilder.getShooterAutoSetCommand(
  //             shooter, drivetrain::getHubToTurretCenterDistanceMeters, () -> 0);
  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getIndexToShooterOnceCommand(indexer, feeder, servo,
  // shooter);

  //     return get3BallAuto(drivetrain, vision, intake, intakeArm, indexer, feeder, servo, shooter)
  //         .andThen(
  //             fourthAndFifthBallPath.alongWith(intakeBalls, setShooter),
  //             new WaitCommand(3),
  //             scoreBallPath.alongWith(setShooter),
  //             feedToShooterOnce,
  //             feedToShooterOnce);
  //   }

  public static Command setLEDsAutoCommand(Vision vision) {
    return new InstantCommand(() -> vision.setForceLeds(false));
  }
}
