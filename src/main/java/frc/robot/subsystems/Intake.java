package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX m_talon = new WPI_TalonSRX(kIntakePort);

  public Intake() {}

  public void setVoltage(double voltage) {
    m_talon.setVoltage(voltage);
  }
}
