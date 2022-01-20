// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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

  @Config(name = "Bang Bang Controller")
  private final BangBangController m_flywheelController = new BangBangController(kVelocityToleranceRotationsPerSecond);

  @Log(name = "Desired Velocity rotations per second")
  private double m_desiredVelocityRPS;

  /** Creates a new Shooter. */
  public Flywheel() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    m_leftTalon.configAllSettings(flywheelTalonConfig);
    try{
      m_leftTalon.setNeutralMode(NeutralMode.Coast);
    } catch(Exception e) {
      DriverStation.reportError("Did not set flywheel neutral mode. Bang bang controller is dangerous in brake mode.", e.getStackTrace());
    }
    
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

  // @Log(name = "Current Kalman filter Velocity (RPS)")
  // public double getPredictedVelocity() {
  //   return m_loop.getObserver().getXhat(0);
  // }

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
    SmartDashboard.putNumber("bangbang output", m_flywheelController.calculate(getRevVelocity(), m_desiredVelocityRPS));
    SmartDashboard.putNumber("feedforward output", m_flywheelFeedforward.calculate(m_desiredVelocityRPS));

    double voltage = kFeedforwardShrink*m_flywheelFeedforward.calculate(m_desiredVelocityRPS) + m_flywheelController.calculate(getRevVelocity(), m_desiredVelocityRPS);
    m_leftTalon.setVoltage(voltage);
  }
}
