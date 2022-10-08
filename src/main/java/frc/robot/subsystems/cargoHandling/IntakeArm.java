// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeArmConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeArm extends SubsystemBase implements Loggable {

  DoubleSolenoid m_solenoid;
  DoubleSolenoid m_solenoid2;
  // boolean armUp = true;

  PneumaticsControlModule m_PCM = new PneumaticsControlModule();
  // Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /** Creates a new intakeArm. */
  public IntakeArm() {

    m_PCM.clearAllStickyFaults();

    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    m_solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    m_solenoid.set(Value.kForward);
    m_solenoid2.set(Value.kForward);
  }

  @Log(name = "moduleID")
  public int moduleNumber() {
    return m_PCM.getModuleNumber();
  }

  // kForward = arm up, kReverse = arm down

  @Log(name = "current position")
  public int getPosition() {
    if (m_solenoid.get() == Value.kForward) {
      return 0;
    } else if (m_solenoid.get() == Value.kReverse) {
      return 1;
    } else {
      return -1;
    }
  }

  public void togglePosition() {
    m_solenoid.toggle();
    m_solenoid2.toggle();
  }

  public void setArmUp() {
    // if (!armUp) {
    m_solenoid.set(Value.kForward);
    m_solenoid2.set(Value.kForward);
    // armUp = true;
    // }
  }

  public void setArmDown() {
    // if (armUp) {
    m_solenoid.set(Value.kReverse);
    m_solenoid2.set(Value.kReverse);
    // armUp = false;
    // }
  }

  @Override
  public void periodic() {}
}
