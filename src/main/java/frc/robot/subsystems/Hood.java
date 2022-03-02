package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;

public class Hood extends SubsystemBase   {

    private CANSparkMax m_spark;
    private Encoder m_hoodEncoder; 
    private SparkMaxLimitSwitch m_hoodLimitSwitch;

}

public Hood (Encoder hoodEncoder){
    
    this.m_spark = new CANSparkMax(Constants.HoodConstants.SPARK_PORT, MotorType.kBrushless);
    this.m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    m_hoodLimitSwitch = m_spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    m_spark.setInverted(false);

    //this.m_spark.getEncoder().setPositionConversionFactor(Constants.HoodConstants.HOOD_GEARING); --> this doesn't make sense

    this.m_hoodEncoder = hoodEncoder;
    m_hoodLimitSwitch.enableLimitSwitch(getBottomLimitSwitch());

}

public boolean getBottomLimitSwitch() {
    return m_hoodLimitSwitch.isPressed();
}

public void setPWM(double hood_pwm) {
    m_spark.set(hood_pwm);
}

@Override
public void runVelocityPID(){
    //velocity controller and whatnot
}

@Override
public void periodic() {  
}

