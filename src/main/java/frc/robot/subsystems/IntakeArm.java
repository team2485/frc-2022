// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
//import sensorPorts;

public class IntakeArm extends SubsystemBase {

  private WL_Sparkmax armSparkMax; //pushin p
  private DigitalInput topSwitch;
  private DigitalInput bottomSwitch; 
  private double pwm;                   //constant to run the intake arm motors up or down ~Yuvi
  /** Creates a new intakeArm. */
  public IntakeArm() {
    topSwitch = new DigitalInput(TOP_SWITCH_PORT); //we put in random ports for now
    bottomSwitch = new DigitalInput(BOTTOM_SWITCH_PORT);
    armSparkMax = new WL_SparkMax(ARM_SPARKMAX_PORT); //pushin p pushin p
    pwm = 0.5;
  }

  public void setUp(){
    if (topSwitch.get()){
      m_armSparkMax.set(0);
    }
    else {
      m_armSparkMax.set(pwm);
    }
  }

  public void setDown(){
    if (bottomSwitch.get()){
      m_armSparkMax.set(0);
    }
    else {
      m_armSparkMax.set(-pwm);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
