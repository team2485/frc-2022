package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;

public class Indexer extends SubsystemBase implements Loggable {
  // Spark max controlling a NEO 550
  private WL_SparkMax m_lowSpark = new WL_SparkMax(kLowIndexerSparkPort);
  private WL_SparkMax m_highSpark = new WL_SparkMax(kHighIndexerSparkPort);

  public Indexer() {
    m_lowSpark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_lowSpark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_lowSpark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_lowSpark.setIdleMode(IdleMode.kBrake);

    m_highSpark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_highSpark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_highSpark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_highSpark.setIdleMode(IdleMode.kBrake);
  }

  public void runLowSpark(double percentOutput) {
    m_lowSpark.set(percentOutput);
  }

  public void runHighSpark(double percentOutput) {
    m_highSpark.set(percentOutput);
  }
}
