// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import sensorPorts;
import com.revrobotics.SparkMaxLimitSwitch;

public class IntakeArm extends SubsystemBase {

  private CANSparkMax m_armSparkMax; //pushin p
  private SparkMaxLimitSwitch m_topSwitch;
  private SparkMaxLimitSwitch m_bottomSwitch;
  

  /** Creates a new intakeArm. */
  public IntakeArm() {

    //pushin p

    m_armSparkMax = new CANSparkMax(IntakeArmConstants.ARM_SPARKMAX_PORT, MotorType.kBrushless);
    m_topSwitch = m_armSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_bottomSwitch = m_armSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    /* limit switches enabled by defualt
       to disable...

       m_topSwitch.enableLimitSwitch(false);
       m_bottomSwitch.enableLimitSwitch(false); */
  }


  public enum IntakePositionEnum{
    up, 
    down;
  }

  public void set(IntakePositionEnum pos){

    if (pos == IntakePositionEnum.up){
      //PWM.UP;
      m_armSparkMax.set(m_topSwitch.isPressed()?0:IntakeArmConstants.PWM_UP);
    }
    else {
      m_armSparkMax.set(m_bottomSwitch.isPressed() ? 0 : IntakeArmConstants.PWM_DOWN);
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Limit Switch", m_topSwitch.isPressed());
    SmartDashboard.putBoolean("Bottom Limit Switch", m_bottomSwitch.isPressed());
  }
}
