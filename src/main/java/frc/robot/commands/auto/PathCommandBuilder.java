package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class PathCommandBuilder {
  public static Command getPathCommand(Drivetrain drivetrain, String name) {
    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            name, kAutoMaxSpeedMetersPerSecond, kAutoMaxAccelerationMetersPerSecondSquared);

    // put trajectory on Glass's Field2d widget
    drivetrain.getField2d().getObject("traj").setTrajectory(path);

    // create controller for robot angle
    var thetaController =
        new ProfiledPIDController(kPAutoThetaController, 0.5, 0, kAutoThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // create command to follow path
    WL_SwerveControllerCommand pathCommand =
        new WL_SwerveControllerCommand(
            path,
            drivetrain::getPoseMeters,
            kDriveKinematics,
            new PIDController(kPAutoXController, 0, 0),
            new PIDController(kPAutoYController, 0, 0),
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

    InstantCommand resetOdometry =
        new InstantCommand(
            () ->
                drivetrain.resetOdometry(
                    new Pose2d(
                        pathCommand.m_trajectory.getInitialState().poseMeters.getTranslation(),
                        pathCommand.m_trajectory.getInitialState().holonomicRotation),
                    false),
            drivetrain);

    InstantCommand stop = new InstantCommand(() -> drivetrain.drive(0, 0, 0, false));
    return resetOdometry.andThen(pathCommand, stop);
  }
}
