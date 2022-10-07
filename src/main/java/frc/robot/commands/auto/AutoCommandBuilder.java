package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.CargoHandlingCommandBuilder.*;
import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;

public class AutoCommandBuilder {

  public static Command get3BallFenderAutoRight(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter) {

    WL_SwerveControllerCommand intakePathCommand =
        getPathCommand(drivetrain, "3 Ball Right Fender");

    WL_SwerveControllerCommand scorePathCommand =
        getPathCommand(drivetrain, "3 Score Right Fender");

    return new WaitCommand(0.5)
        .andThen(getIndexToShooterCommand(indexer, feeder, servo), new WaitCommand(0.5))
        .raceWith(getSetShooterCommand(shooter))
        .andThen(
            getResetOdometryCommand(drivetrain, intakePathCommand),
            new InstantCommand(
                () ->
                    drivetrain
                        .getField2d()
                        .getObject("traj")
                        .setTrajectory(intakePathCommand.m_trajectory),
                drivetrain),
            intakePathCommand
                .andThen(getStopPathCommand(drivetrain), new WaitCommand(1.5))
                .raceWith(getIntakeCommand(intake, intakeArm, indexer, servo)),
            new ParallelDeadlineGroup(
                new InstantCommand(
                        () ->
                            drivetrain
                                .getField2d()
                                .getObject("traj")
                                .setTrajectory(scorePathCommand.m_trajectory),
                        drivetrain)
                    .andThen(scorePathCommand, getStopPathCommand(drivetrain)),
                getStopIntakeCommand(intake, intakeArm, indexer)),
            new ParallelRaceGroup(
                getIndexToShooterCommand(indexer, feeder, servo)
                    .andThen(getIndexToShooterCommand(indexer, feeder, servo))
                    .raceWith(getSetShooterCommand(shooter))));
  }

  public static Command get2BallFenderAutoLeft(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Timer timer,
      Hood hood) {

    WL_SwerveControllerCommand pathCommand = getPathCommand(drivetrain, "2 Ball Left Fender");

    return new InstantCommand(
            () -> {
              timer.reset();
              timer.start();
            })
        .andThen(
            new WaitCommand(0.5),
            new InstantCommand(()->hood.setAngleRadians(0.1)),
            getResetOdometryCommand(drivetrain, pathCommand),
            new InstantCommand(
                () ->
                    drivetrain
                        .getField2d()
                        .getObject("traj")
                        .setTrajectory(pathCommand.m_trajectory),
                drivetrain),
            pathCommand
                .withInterrupt(() -> timer.get() > 8)
                .andThen(getStopPathCommand(drivetrain), new WaitCommand(2))
                .raceWith(runTestCommand(intake, intakeArm, indexer)),
            stopTestCommand(intake, intakeArm, indexer),
            getRunFeederCommand(feeder, indexer))
        .alongWith(getSetShooterCommand(shooter));
  }

  //   public static Command getSwordfishAuto() {
  //     return new RunCommand(()-> m_intak.)
  //   }

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
