// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.FlywheelConstants.*;

public class Flywheel extends SubsystemBase {
  private final WPI_TalonFX m_leftTalon = new WPI_TalonFX(kLeftTalonPort);
 // private final WPI_TalonFX m_rightTalon = new WPI_TalonFX(kRightTalonPort);
  
  private final Encoder m_encoder = new Encoder(kFlywheeEncoderPort1, kFlywheelEncoderPort2);

  private final SimpleMotorFeedforward m_flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

  @Config(name = "Flywheel PID controller")
  private final PIDController m_flywheelController = new PIDController(kP, 0, kD);

  /** Creates a new Shooter. */
  public Flywheel() {
    m_leftTalon.setNeutralMode(NeutralMode.Coast);
   // m_rightTalon.setNeutralMode(NeutralMode.Coast);
    //m_rightTalon.setInverted(true);
    m_encoder.setDistancePerPulse(1/kRevEncoderPulsesPerRevolution);
  }
  
  @Log(name = "Velocity")
  public double getVelocity() {
    return m_encoder.getRate();
  }

  @Config(name = "Velocity Rotations Per Second")
  public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
    double voltage = m_flywheelFeedforward.calculate(rotationsPerSecond) + m_flywheelController.calculate(getVelocity(), rotationsPerSecond);
    m_leftTalon.setVoltage(voltage);
    //m_rightTalon.setVoltage(voltage);
  }

  public void setVoltage(double input) {
    m_leftTalon.setVoltage(input);
    //m_rightTalon.setVoltage(input);
  }
}
