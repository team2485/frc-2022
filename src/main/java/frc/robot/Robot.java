// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.drive.CTREConfigs;
import frc.robot.subsystems.drive.Drivetrain;
import frc.WarlordsLib.IDManager;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static CTREConfigs ctreConfigs;

  Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
  PIDController linearVisionController = new PIDController(VisionConstants.kVisionLinearP, 0, VisionConstants.kVisionLinearD);
  PIDController rotationVisionController = new PIDController(VisionConstants.kVisionAngularP, 0, VisionConstants.kVisionAngularD);

  XboxController xboxController = new XboxController(0);

  Drivetrain drive = new Drivetrain();

  public Robot() {
    ctreConfigs = new CTREConfigs();
    IDManager.getInstance(Constants.kRobotIdFile);
    m_robotContainer = new RobotContainer();
    addPeriodic(
        () -> m_robotContainer.m_climbElevator.runControlLoop(),
        Constants.ClimbElevatorConstants.kElevatorControlLoopTimeSeconds);

    addPeriodic(
        () -> m_robotContainer.m_climbArm.runControlLoop(),
        Constants.ClimbArmConstants.kArmControlLoopTimeSeconds);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    LiveWindow.disableAllTelemetry();
    VideoSource video = CameraServer.startAutomaticCapture();
    video.setVideoMode(new VideoMode(PixelFormat.kMJPEG, 320, 240, 120));
    video.setFPS(5);

    // Make the robot container the root project for Oblog
    Logger.configureLoggingAndConfig(m_robotContainer, false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    NetworkTableInstance.getDefault().flush();
    // System.out.println("Potentiometer reading: " + m_potentiometer.get());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_compressor.disable();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.configureDriveCoastMode();

    m_compressor.disable();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double xSpeed;
    double ySpeed;
    double rotationSpeed;

    if (xboxController.getAButton()) {
      // vision-alignment mode

      PhotonPipelineResult result = camera.getLatestResult();

      if (result.hasTargets()) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kLensHeightMeters, VisionConstants.kTargetHeightMeters, VisionConstants.kLensPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
        // double offset = PhotonUtils.
        xSpeed = 0;
        ySpeed = -linearVisionController.calculate(range, VisionConstants.kGoalRangeMeters);
        rotationSpeed = -rotationVisionController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        xSpeed = 0;
        ySpeed = 0;
        rotationSpeed = 0;
      }
    } else {
      xSpeed = -xboxController.getLeftX();
      ySpeed = -xboxController.getLeftY();
      rotationSpeed = -xboxController.getRightX();
    }

    /* Get Values, Deadband*/
    double translationVal = MathUtil.applyDeadband(ySpeed, Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(xSpeed, Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSpeed, Constants.stickDeadband);

    /* Drive */
    drive.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        xboxController.getRightBumper(), 
        true
    );
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance().enable();
    // m_robotContainer.configureDriveCoastMode();

    m_compressor.enableDigital();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // m_robotContainer.testPeriodic();
  }
}
