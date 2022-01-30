// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.BangBangController;
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
import static frc.robot.Constants.*;


public class Flywheel extends SubsystemBase implements Loggable {
  private final WPI_TalonFX m_talon = new WPI_TalonFX(kLeftTalonPort);
  
  // private final Encoder m_encoder = new Encoder(kFlywheelEncoderPort1, kFlywheelEncoderPort2);

  private final SimpleMotorFeedforward m_flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  private final BangBangController m_bangBangController = new BangBangController(kVelocityTolerance);

  private double m_desiredVelocityRPS;

  /** Creates a new Shooter. */
  public Flywheel() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.slot0.kP = kP;
    flywheelTalonConfig.slot0.kD = kD;
    m_talon.configAllSettings(flywheelTalonConfig);

    m_talon.setNeutralMode(NeutralMode.Coast);
   // m_rightTalon.setNeutralMode(NeutralMode.Coast);
    //m_rightTalon.setInverted(true);
    // m_encoder.setReverseDirection(true);
    // m_encoder.setDistancePerPulse(1.0/kRevEncoderPulsesPerRevolution);
    // m_encoder.setSamplesToAverage(kRevEncoderSamplesToAverage);
  }
  
  // @Log(name = "Current Rev-reported Velocity (RPS)")
  // public double getRevVelocity() {
  //   return m_encoder.getRate();
  // }

  @Log(name = "Current Talon-reported Velocity (RPS)")
  public double getTalonVelocity() {
    return m_talon.getSelectedSensorVelocity() * kFlywheelRotationsPerPulse * 10;
  }

  @Config(name = "Set Velocity (RPS)")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    m_desiredVelocityRPS = rotationsPerSecond;
    //m_rightTalon.setVoltage(voltage);
  }

  public void setVoltage(double input) {
    m_talon.setVoltage(input);
    //m_rightTalon.setVoltage(input);
  }

  @Override
  public void periodic() {
    double voltage = 0.95 * m_flywheelFeedforward.calculate(m_desiredVelocityRPS)
     + m_bangBangController.calculate(getTalonVelocity(), m_desiredVelocityRPS) * kNominalVoltage;
    m_talon.setVoltage(voltage);
    
    SmartDashboard.putNumber("ff applied voltage", voltage);
    SmartDashboard.putNumber("talon applied voltage", m_talon.getBusVoltage());

}
}
