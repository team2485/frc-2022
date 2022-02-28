package frc.robot.subsystems.cargoHandling.indexing;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class HighIndexer extends SubsystemBase implements Loggable {
  private CANSparkMax m_spark = new CANSparkMax(kHighIndexerSparkPort, MotorType.kBrushless);

  public HighIndexer() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
  }

  @Config.NumberSlider(name = "Set percent output", tabName = "Indexing")
  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }
}
