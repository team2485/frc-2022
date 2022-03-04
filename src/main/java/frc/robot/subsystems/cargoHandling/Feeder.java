package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Feeder extends SubsystemBase implements Loggable {
  private WL_SparkMax m_spark = new WL_SparkMax(kHighIndexerSparkPort);

  private Servo m_servo = new Servo(1);
  private double m_servoPositionSetpoint = 0;

  public Feeder() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
  }

  @Config.NumberSlider(name = "Set percent output high")
  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }

  public void setServo(double position) {
    m_servoPositionSetpoint = position;
  }

  @Config.ToggleButton(name = "Set servo")
  public void engageServo(boolean engaged) {
    if (engaged) {
      m_servoPositionSetpoint = kServoEngagedPosition;
    } else {
      m_servoPositionSetpoint = kServoDisengagedPosition;
    }
  }

  public void periodic() {
    m_servo.set(m_servoPositionSetpoint);
  }
}
