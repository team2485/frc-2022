// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class IntakeArm extends SubsystemBase implements Loggable {

  DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  boolean armUp = true;

  /** Creates a new intakeArm. */
  public IntakeArm() {

    m_solenoid.set(Value.kReverse);
  }

  public void togglePosition() {
    m_solenoid.toggle();
  }

  public void setArmUp() {
    if (!armUp) {
      m_solenoid.set(Value.kReverse);
      armUp = true;
    }
  }

  public void setArmDown() {
    if (armUp) {
      m_solenoid.set(Value.kForward);
      armUp = false;
    }
  }

  @Override
  public void periodic() {}
}
