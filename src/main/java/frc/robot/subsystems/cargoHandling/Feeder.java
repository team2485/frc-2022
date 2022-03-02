package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.FeederConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;

public class Feeder extends SubsystemBase implements Loggable {
  private WL_SparkMax m_spark = new WL_SparkMax(kFeederSparkPort);

  public Feeder() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kFeederSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kFeederImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
  }

  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }
}
