// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_leftTalon = new WPI_TalonFX(kLeftTalonPort);
 // private final WPI_TalonFX m_rightTalon = new WPI_TalonFX(kRightTalonPort);
  
  private final Encoder m_encoder = new Encoder(kFlywheeEncoderPort1, kFlywheelEncoderPort2);

  private final SimpleMotorFeedforward m_flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
  LinearSystemId.identifyVelocitySystem(kV, kA);

  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
    Nat.N1(),
    Nat.N1(),
    m_flywheelPlant,
    VecBuilder.fill(3.0), // How accurate we think our model is
    VecBuilder.fill(0.01), // How accurate we think our encoder
    // data is
    0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
    new LinearQuadraticRegulator<>(
        m_flywheelPlant,
        VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
        // this to more heavily penalize state excursion, or make the controller behave more
        // aggressively.
        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
        // starting point because that is the (approximate) maximum voltage of a battery.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

      // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
  new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  @Config(name = "Flywheel PID controller")
  private final PIDController m_flywheelController = new PIDController(kP, 0, kD);

  private double m_desiredVelocityRPS;

  /** Creates a new Shooter. */
  public Flywheel() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    m_leftTalon.configAllSettings(flywheelTalonConfig);

    m_leftTalon.setNeutralMode(NeutralMode.Coast);
    m_leftTalon.setInverted(true);
   // m_rightTalon.setNeutralMode(NeutralMode.Coast);
    //m_rightTalon.setInverted(true);
    m_encoder.setReverseDirection(true);
    m_encoder.setDistancePerPulse(1.0/kRevEncoderPulsesPerRevolution);
    m_encoder.setSamplesToAverage(kRevEncoderSamplesToAverage);
  }
  
  @Log(name = "Current Rev-reported Velocity (RPS)")
  public double getRevVelocity() {
    return m_encoder.getRate();
  }

  @Log(name = "Current Kalman filter Velocity (RPS)")
  public double getPredictedVelocity() {
    return m_loop.getObserver().getXhat(0);
  }

  @Log(name = "Current Talon-reported Velocity (RPS)")
  public double getTalonVelocity() {
    return m_leftTalon.getSelectedSensorVelocity() * kFlywheelRotationsPerPulse * 10;
  }

  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_desiredVelocityRPS = rotationsPerSecond;
    //m_rightTalon.setVoltage(voltage);
  }

  public void setVoltage(double input) {
    m_leftTalon.setVoltage(input);
    //m_rightTalon.setVoltage(input);
  }

  @Override
  public void periodic() {
    // double voltage = m_flywheelFeedforward.calculate(m_desiredVelocityRPS) + m_flywheelController.calculate(getRevVelocity(), m_desiredVelocityRPS);
    // m_leftTalon.setVoltage(voltage);
    
    m_loop.setNextR(VecBuilder.fill(m_desiredVelocityRPS));

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    //save battery and mechanism strain by not applying voltage backward
    if(nextVoltage < 0) {
      nextVoltage = 0;
    }
    SmartDashboard.putNumber("loop applied voltage", nextVoltage);
    m_leftTalon.setVoltage(nextVoltage);

  }
}
