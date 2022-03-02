package frc.robot.subsystems.cargoHandling.indexing;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;

public class HighIndexer extends SubsystemBase implements Loggable {
  private WL_SparkMax m_spark = new WL_SparkMax(kHighIndexerSparkPort);

  public HighIndexer() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
  }

  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }
}