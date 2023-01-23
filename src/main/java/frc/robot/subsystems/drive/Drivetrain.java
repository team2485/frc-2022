package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.vision.TargetVision;
import frc.util.photonVision.EstimatedRobotPose;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public WPI_Pigeon2 gyro;

  private final TargetVision m_camera = new TargetVision();
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.swerveKinematics,
      gyro.getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.1, 0.1, 0.1));

  private final Field2d m_fieldSim = new Field2d();

  public Drivetrain() {
    gyro = new WPI_Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    SmartDashboard.putData("Field", m_fieldSim);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose, Rotation2d rotation) {
    swerveOdometry.resetPosition(rotation, getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public void simulationInit() {
    Pose2d startPos = new Pose2d(0, 0, new Rotation2d(0));
    m_fieldSim.setRobotPose(startPos);
    resetOdometry(m_fieldSim.getRobotPose(), startPos.getRotation());
    m_fieldSim.getObject("trajectory").setPose(new Pose2d());
  }

  public void simulationPeriodic() {
    m_fieldSim.setRobotPose(getPose());
  }

  public void updateOdometry() {
    m_poseEstimator.update(gyro.getRotation2d(), getModulePositions());

    Optional<EstimatedRobotPose> result = m_camera.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      m_fieldSim.getObject("Camera Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
      m_fieldSim.getObject("Camera Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    m_fieldSim.getObject("Actual Pos").setPose(getPose());
    m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }
}
