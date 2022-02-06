package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Hood;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;


public class Hood extends SubsystemBase implements PositionPIDSubsystem, VelocityPIDSubsystem {

    private WL_SparkMax m_spark;
    private Encoder m_hoodEncoder; 
    private SparkMaxLimitSwitch m_hoodLimitSwitch;

    private WL_PIDController m_velocityController;

    private WL_PIDController m_positionController;

    private boolean m_isZeroed = false; 

}

public Hood (Encoder hoodEncoder){
    this.m_spark = new WL_SparkMax(Constants.Hood.SPARK_PORT);
    this.m_spark.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);

    m_spark.setInverted(false);

    //do this
    this.m_spark.getBottomLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);

    this.m_spark.getEncoder().setPositionConversionFactor(Constants.Hood.HOOD_GEARING);
    this.m_hoodEncoder.setVelocityConversionFactor(Constants.Hood.HOOD_GEARING);

    this.m_hoodEncoder = hoodEncoder;

}

public boolean getBottomLimitSwitch() {
    return m_hoodLimitSwitch.isPressed();
}