// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.sendableRichness.SR_TrapezoidProfile;
import io.github.oblarg.oblog.annotations.*;

public class Turret extends SubsystemBase {
  private final WPI_TalonSRX m_talon = new WPI_TalonSRX(kTurretTalonPort);
  private final AnalogPotentiometer m_potentiometer =
      new AnalogPotentiometer(kTurretEncoderChannel, kTurretRangeOfMotion, kTurretOffset);

  private SR_TrapezoidProfile.State m_lastProfiledReference = new SR_TrapezoidProfile.State();

  // The plant holds a state-space model of our turret. This system has the following properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N1> m_turretPlant =
      LinearSystemId.identifyPositionSystem(
          kVTurretVoltSecondsPerMeter, kATurretVoltSecondsSquaredPerMeter);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_turretPlant,
          VecBuilder.fill(0, kTurretVelocityRMSE), // How accurate we
          // think our model is, in radians and radians/sec
          VecBuilder.fill(kTurretPotentiometerStdDev), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          kTurretLoopTimeSeconds);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_turretPlant,
          VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
          // Position and velocity error tolerances, in radians and radians per second. Decrease
          // this
          // to more heavily penalize state excursion, or make the controller behave more
          // aggressively..
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          kTurretLoopTimeSeconds); // Nominal time between loops.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(
          m_turretPlant,
          m_controller,
          m_observer,
          Constants.kNominalVoltage,
          kTurretLoopTimeSeconds);

  // Angle coord system is 0 forward, - CW, + CCW
  private double m_angleSetpointRadians = 0;

  private double m_lastAngleRadians = 0;
  private double m_velocityRadiansPerSecond = 0;

  public Turret() {

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.voltageCompSaturation = Constants.kNominalVoltage;
    talonConfig.peakCurrentLimit = kTurretSupplyCurrentThresholdAmps;
    talonConfig.peakCurrentDuration = kTurretSupplyCurrentThresholdTimeMs;
    talonConfig.continuousCurrentLimit = kTurretSupplyCurrentLimitAmps;

    m_talon.configAllSettings(talonConfig);
    m_talon.enableVoltageCompensation(true);
    m_talon.setNeutralMode(NeutralMode.Brake);

    m_talon.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_talon.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(this.getAngleRadians(), this.getVelocityRadiansPerSecond()));

    // Reset our last reference to the current state.
    m_lastProfiledReference =
        new SR_TrapezoidProfile.State(this.getAngleRadians(), this.getVelocityRadiansPerSecond());
  }

  @Config(name = "Set angle (radians)")
  public void setAngleRadians(double angle) {
    m_angleSetpointRadians =
        MathUtil.clamp(angle % (2 * Math.PI), kMinPositionRadians, kMaxPositionRadians);
  }

  @Log(name = "Current angle (radians)")
  public double getAngleRadians() {
    return m_potentiometer.get() % (2 * Math.PI);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(this.getAngleRadians());
  }

  public double getVelocityRadiansPerSecond() {
    return m_velocityRadiansPerSecond;
  }

  @Log(name = "Counter-clockwise limit switch")
  private boolean getCCWLimitSwitchEnabled() {
    return m_talon.isFwdLimitSwitchClosed() == 1 ? true : false;
  }

  @Log(name = "Clockwise limit switch")
  private boolean getCWLimitSwitchEnabled() {
    return m_talon.isRevLimitSwitchClosed() == 1 ? true : false;
  }

  public void runControlLoop() {
    double currentAngleRadians = this.getAngleRadians();
    m_velocityRadiansPerSecond =
        (currentAngleRadians - m_lastAngleRadians) / kTurretLoopTimeSeconds;
    m_lastAngleRadians = currentAngleRadians;
  }

  public void periodic() {
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference =
        (new SR_TrapezoidProfile(
                kTurretMotionProfileConstraints,
                new SR_TrapezoidProfile.State(m_angleSetpointRadians, 0.0),
                m_lastProfiledReference))
            .calculate(0.020);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(this.getAngleRadians()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(kTurretLoopTimeSeconds);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);

    m_talon.set(ControlMode.PercentOutput, nextVoltage / Constants.kNominalVoltage);
  }
}
