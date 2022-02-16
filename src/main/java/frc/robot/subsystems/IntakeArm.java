// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.IntakeArmConstants.*;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import sensorPorts;
import com.revrobotics.SparkMaxLimitSwitch;

public class IntakeArm extends SubsystemBase implements Loggable {

  private WL_SparkMax m_spark;
  private SparkMaxLimitSwitch m_topSwitch;
  private SparkMaxLimitSwitch m_bottomSwitch;
  

  /** Creates a new intakeArm. */
  public IntakeArm() {
    m_spark = new WL_SparkMax(kIntakeArmSparkPort);
    m_spark.setSmartCurrentLimit(kIntakeArmSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIntakeArmImmediateCurrentLimitAmps);
    m_topSwitch = m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_bottomSwitch = m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    m_topSwitch.enableLimitSwitch(true);
    m_bottomSwitch.enableLimitSwitch(true);
  }


  public enum ArmPosition {
    kUp, 
    kDown;
  }

  public void set(ArmPosition pos){

    if (pos == ArmPosition.kUp){
      m_spark.set(kPercentOutputUp);
    }
    else if(pos == ArmPosition.kDown) {
      m_spark.set(kPercentOutputDown);
    }
  }

  @Log (name = "Top Limit Switch")
  public boolean getTopLimitSwitch() {
    return m_topSwitch.isPressed();
  }

  @Log (name = "Bottom Limit Switch")
  public boolean getBottomLimitSwitch() {
    return m_bottomSwitch.isPressed();
  }

}
