package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.CargoHandlingCommandBuilder.*;
import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
import frc.robot.subsystems.cargoHandling.*;
import frc.robot.subsystems.drive.*;

public class AutoCommandBuilder {

  public static Command get5BallAuto(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    WL_SwerveControllerCommand part1 = getPathCommand(drivetrain, "5 Ball Pt. 1");

    WL_SwerveControllerCommand part2 = getPathCommand(drivetrain, "5 Ball Pt. 2");

    WL_SwerveControllerCommand part3 = getPathCommand(drivetrain, "5 Ball Pt. 3");

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(2),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()),
            getResetOdometryCommand(drivetrain, part1),
            new InstantCommand(
                () -> drivetrain.getField2d().getObject("traj").setTrajectory(part1.m_trajectory),
                drivetrain),
            part1
                .alongWith(
                    intakeForAutoCommand(intake, indexer)
                        .withTimeout(5.5)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(5.5),
            getStopPathCommand(drivetrain),
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(2),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()),
            getResetOdometryCommand(drivetrain, part2),
            new InstantCommand(
                () -> drivetrain.getField2d().getObject("traj").setTrajectory(part2.m_trajectory),
                drivetrain),
            new ParallelCommandGroup(
                part2.withTimeout(3),
                new WaitCommand(2)
                    .andThen(
                        intakeForAutoCommand(intake, indexer).withTimeout(2),
                        stopIntakeForAutoCommand(intake, indexer))),
            getStopPathCommand(drivetrain),
            getResetOdometryCommand(drivetrain, part3),
            new InstantCommand(
                () -> drivetrain.getField2d().getObject("traj").setTrajectory(part3.m_trajectory),
                drivetrain),
            part3.withTimeout(4),
            getStopPathCommand(drivetrain),
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(4),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()));
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

    WL_SwerveControllerCommand part1 = getPathCommand(drivetrain, "5 Ball Pt. 1");

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(3),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()),
            getResetOdometryCommand(drivetrain, part1),
            new InstantCommand(
                () -> drivetrain.getField2d().getObject("traj").setTrajectory(part1.m_trajectory),
                drivetrain),
            part1
                .alongWith(
                    intakeForAutoCommand(intake, indexer)
                        .withTimeout(5.5)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(5.5),
            getStopPathCommand(drivetrain),
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(2),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()));
  }

  public static Command get3BallAutoLeft(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    WL_SwerveControllerCommand part1 = getPathCommand(drivetrain, "2 Ball Left Fender");

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(3),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()),
            getResetOdometryCommand(drivetrain, part1),
            new InstantCommand(
                () -> drivetrain.getField2d().getObject("traj").setTrajectory(part1.m_trajectory),
                drivetrain),
            part1
                .alongWith(
                    intakeForAutoCommand(intake, indexer)
                        .withTimeout(3)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(5),
            getStopPathCommand(drivetrain),
            setShooterForShot(hood, shooter),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(2),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()));
  }











  
  public static Command get3BallFenderAutoRight(
      Drivetrain drivetrain,
      Intake intake,
      IntakeArm intakeArm,
      Indexer indexer,
      Feeder feeder,
      FeedServo servo,
      Shooter shooter,
      Hood hood) {

    WL_SwerveControllerCommand intakePathCommand =
        getPathCommand(drivetrain, "3 Ball Right Fender");

    WL_SwerveControllerCommand scorePathCommand =
        getPathCommand(drivetrain, "3 Score Right Fender");

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            new WaitCommand(0.75),
            getResetOdometryCommand(drivetrain, intakePathCommand),
            new InstantCommand(
                () ->
                    drivetrain
                        .getField2d()
                        .getObject("traj")
                        .setTrajectory(intakePathCommand.m_trajectory),
                drivetrain),
            intakePathCommand
                .alongWith(
                    intakeForAutoCommand(intake, indexer)
                        .withTimeout(2)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(3)
                .andThen(
                    getStopPathCommand(drivetrain),
                    setShooterForShot(hood, shooter),
                    getSetShooterCommand(shooter)
                        .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                        .withTimeout(2),
                    getStopFeederCommand(feeder, indexer),
                    new InstantCommand(() -> shooter.zeroShooter()),
                    new InstantCommand(
                        () ->
                            drivetrain
                                .getField2d()
                                .getObject("traj")
                                .setTrajectory(scorePathCommand.m_trajectory),
                        drivetrain),
                    scorePathCommand
                        .alongWith(
                            intakeForAutoCommand(intake, indexer)
                                .withTimeout(2.5)
                                .andThen(stopIntakeForAutoCommand(intake, indexer)))
                        .withTimeout(3)
                        .andThen(
                            getStopPathCommand(drivetrain),
                            CargoHandlingCommandBuilder.setShooterForShot(hood, shooter),
                            getSetShooterCommand(shooter)
                                .alongWith(
                                    new WaitCommand(1)
                                        .andThen(getRunFeederCommand(feeder, indexer)))
                                .withTimeout(2.5),
                            getStopFeederCommand(feeder, indexer),
                            new InstantCommand(() -> shooter.zeroShooter()),
                            new InstantCommand(() -> intakeArm.setArmUp()))));
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

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            new WaitCommand(0.5),
            getResetOdometryCommand(drivetrain, pathCommand),
            new InstantCommand(
                () ->
                    drivetrain
                        .getField2d()
                        .getObject("traj")
                        .setTrajectory(pathCommand.m_trajectory),
                drivetrain),
            pathCommand
                .alongWith(
                    intakeForAutoCommand(intake, indexer)
                        .withTimeout(2)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(4.75)
                .andThen(getStopPathCommand(drivetrain)),
            CargoHandlingCommandBuilder.setShooterForShot(hood, shooter),
            new WaitCommand(1),
            getSetShooterCommand(shooter)
                .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                .withTimeout(3),
            getStopFeederCommand(feeder, indexer),
            new InstantCommand(() -> shooter.zeroShooter()),
            new InstantCommand(() -> intakeArm.setArmUp()));
  }
}
