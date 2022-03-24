package frc.robot.commands.auto;

import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;

public class AutoCommandBuilder {

  //   public static Command get2BallAutoRight(
  //       Drivetrain drivetrain,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter,
  //       Hood hood) {

  //     Command backUpPath = PathCommandBuilder.getPathCommand(drivetrain, "2 Ball Right");
  //     Command scoochOverPath = PathCommandBuilder.getPathCommand(drivetrain, "2 Ball Right
  // Finish");
  //     Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);
  //     Command stopIntake =
  //         CargoHandlingCommandBuilder.getStopIntakeCommand(intake, intakeArm, indexer);
  //     Command setShooter = CargoHandlingCommandBuilder.getShooterSetCommand(shooter, () -> 123);

  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getSetHoodCommand(() -> 0.475, hood)
  //             .withTimeout(0.5)
  //             .andThen(
  //                 CargoHandlingCommandBuilder.getIndexToShooterCommand(
  //                     indexer, feeder, servo, shooter));

  //     Command feedToShooterOnce2 =
  //         CargoHandlingCommandBuilder.getIndexToShooterCommand(indexer, feeder, servo, shooter);

  //     return new ParallelRaceGroup(
  //             new WaitCommand(0.5).andThen(backUpPath, new WaitCommand(2)),
  //             intakeBalls.alongWith(setShooter))
  //         .andThen(stopIntake)
  //         .andThen(feedToShooterOnce, feedToShooterOnce2)
  //         .andThen(scoochOverPath);
  //   }

  //   public static Command get2BallAutoLeft(
  //       Drivetrain drivetrain,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter,
  //       Hood hood) {

  //     Command backUpPath = PathCommandBuilder.getPathCommand(drivetrain, "2 Ball Left");

  //     Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);
  //     Command stopIntake =
  //         CargoHandlingCommandBuilder.getStopIntakeCommand(intake, intakeArm, indexer);
  //     Command setShooter = CargoHandlingCommandBuilder.getSetShooterCommand(() -> 123, shooter);

  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getSetHoodCommand(() -> 0.475, hood)
  //             .withTimeout(0.9)
  //             .andThen(
  //                 CargoHandlingCommandBuilder.getIndexToShooterCommand(
  //                     indexer, feeder, servo, shooter));

  //     Command feedToShooterOnce2 =
  //         CargoHandlingCommandBuilder.getIndexToShooterCommand(indexer, feeder, servo, shooter);

  //     return new ParallelRaceGroup(
  //             new WaitCommand(0.5).andThen(backUpPath, new WaitCommand(3)),
  //             intakeBalls.alongWith(setShooter))
  //         .andThen(stopIntake)
  //         .andThen(feedToShooterOnce, feedToShooterOnce2);
  //   }

  //   public static Command get3BallAuto(
  //       Drivetrain drivetrain,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter,
  //       Hood hood) {

  //     Command thirdBallPath = PathCommandBuilder.getPathCommand(drivetrain, "2 to 3 ball right");

  //     Command intakeBalls =
  //         CargoHandlingCommandBuilder.getIntakeCommand(intake, intakeArm, indexer, servo);
  //     Command setShooter =
  //         CargoHandlingCommandBuilder.getShooterAutoSetCommand(
  //             shooter,
  //             drivetrain::getHubToTurretCenterDistanceMeters,
  //             () -> {
  //               return 0;
  //             });
  //     Command feedToShooterOnce =
  //         CargoHandlingCommandBuilder.getIndexToShooterCommand(indexer, feeder, servo, shooter);

  //     return get2BallAutoRight(drivetrain, intake, intakeArm, indexer, feeder, servo, shooter,
  // hood)
  //         .andThen(thirdBallPath.alongWith(intakeBalls, setShooter), feedToShooterOnce);
  //   }

  //   public static Command get4BallAuto(
  //       Drivetrain drivetrain,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter,
  //       Hood hood) {

  //     Command thirdAndFourthBallPath =
  //         PathCommandBuilder.getPathCommand(drivetrain, "2 to 4 ball right");

  //     Command scoreBallPath = PathCommandBuilder.getPathCommand(drivetrain, "4 or 5 to score
  // right");

  //     Command intakeBalls = getIntakeBallsCommand(intake, intakeArm, indexer, servo);

  //     Command setShooter = getSetShooterCommand(shooter, drivetrain);

  //     Command feedToShooterOnce =
  //         getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

  //     Command feedToShooterOnce2 =
  //         getFeedToShooterCommand(hood, drivetrain, indexer, feeder, servo, shooter);

  //     return get2BallAutoRight(drivetrain, intake, intakeArm, indexer, feeder, servo, shooter,
  // hood)
  //         .andThen(
  //             new ParallelRaceGroup(
  //                 new WaitCommand(0.2).andThen(thirdAndFourthBallPath, new WaitCommand(2)),
  //                 intakeBalls,
  //                 new WaitCommand(0.5),
  //                 new ParallelRaceGroup(scoreBallPath.andThen(new WaitCommand(0.2)), setShooter),
  //                 feedToShooterOnce,
  //                 feedToShooterOnce2));
  //   }

  //   public static Command getEpicMemerAuto(
  //       Drivetrain drivetrain,
  //       Intake intake,
  //       IntakeArm intakeArm,
  //       Indexer indexer,
  //       Feeder feeder,
  //       FeedServo servo,
  //       Shooter shooter,
  //       Hood hood) {
  //     Command epicMemerPath = PathCommandBuilder.getPathCommand(drivetrain, "Epic Memer");
  //     Command intakeArmUp = CargoHandlingCommandBuilder.getIntakeArmUpCommand(intakeArm);
  //     Command outtakeBall =
  //         new RunCommand(() -> indexer.setVelocityRotationsPerSecond(-20), indexer)
  //             .withTimeout(2)
  //             .andThen(new InstantCommand(() -> indexer.setVelocityRotationsPerSecond(0)));
  //     return epicMemerPath.andThen(intakeArmUp, outtakeBall);
  //   }

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

  //     return get3BallAuto(drivetrain, vision, intake, intakeArm, indexer, feeder, servo,
  // shooter)
  //         .andThen(
  //             fourthAndFifthBallPath.alongWith(intakeBalls, setShooter),
  //             new WaitCommand(3),
  //             scoreBallPath.alongWith(setShooter),
  //             feedToShooterOnce,
  //             feedToShooterOnce);
  //   }

}
