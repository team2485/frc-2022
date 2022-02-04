// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import WarlordsLib.BufferZone;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.TurretConstants;
public class Turret extends SubsystemBase {
  private ProfiledPIDController m_controller = new ProfiledPIDController(
    TurretConstants.KP,
    TurretConstants.KI,
    TurretConstants.KD,
    new TrapezoidProfile.Constraints(TurretConstants.MAX_VELOCITY, TurretConstants.MAX_ACCELERATION)
  );

  private BufferZone m_bufferZone = new BufferZone(TurretConstants.MIN_OUTPUT, TurretConstants.MAX_OUTPUT, TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, TurretConstants.BUFFER_SIZE);

  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(TurretConstants.KS, TurretConstants.KV, TurretConstants.KA);
  private WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.TURRET_MOTOR_PORT);
  private double m_currentVelocity = 0;
  private double m_prevVelocity = 0;
  private double m_currentRotation = 0;

  public Turret() {
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveTowardsRotationSetpoint(double newRotation)
  {
    m_prevVelocity = m_currentVelocity;
    m_currentVelocity = m_controller.calculate(m_currentRotation, newRotation);
    m_motor.set(m_bufferZone.get(m_feedforward.calculate(m_currentVelocity, m_currentVelocity - m_prevVelocity) / TurretConstants.TURRET_VOLTAGE), m_currentRotation);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}