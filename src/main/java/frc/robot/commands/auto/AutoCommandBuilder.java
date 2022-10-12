package frc.robot.commands.auto;

import static frc.robot.Constants.*;
import static frc.robot.commands.CargoHandlingCommandBuilder.*;
import static frc.robot.commands.auto.PathCommandBuilder.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CargoHandlingCommandBuilder;
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
      Shooter shooter,
      Hood hood) {

    WL_SwerveControllerCommand intakePathCommand =
        getPathCommand(drivetrain, "3 Ball Right Fender");

    WL_SwerveControllerCommand scorePathCommand =
        getPathCommand(drivetrain, "3 Score Right Fender");

    return new InstantCommand(() -> intakeArm.setArmDown(), intakeArm)
        .andThen(
            new WaitCommand(1),
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
                        .withTimeout(2.5)
                        .andThen(stopIntakeForAutoCommand(intake, indexer)))
                .withTimeout(4)
                .andThen(
                    getStopPathCommand(drivetrain),
                    CargoHandlingCommandBuilder.setShooterForShot(hood, shooter),
                    new WaitCommand(1),
                    getSetShooterCommand(shooter)
                        .alongWith(new WaitCommand(1).andThen(getRunFeederCommand(feeder, indexer)))
                        .withTimeout(3),
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
                                .withTimeout(4)
                                .andThen(
                                    stopIntakeForAutoCommand(intake, indexer)))
                        .withTimeout(7)
                        .andThen(
                            getStopPathCommand(drivetrain),
                            CargoHandlingCommandBuilder.setShooterForShot(hood, shooter),
                            new WaitCommand(1),
                            getSetShooterCommand(shooter)
                                .alongWith(
                                    new WaitCommand(1)
                                        .andThen(getRunFeederCommand(feeder, indexer)))
                                .withTimeout(3),
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
