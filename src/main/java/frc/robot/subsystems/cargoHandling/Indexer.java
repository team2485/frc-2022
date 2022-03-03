package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Indexer extends SubsystemBase implements Loggable {
  private WL_SparkMax m_spark = new WL_SparkMax(kIndexerSparkPort);

  private double m_lastVelocity;

  private double m_velocitySetpoint;

  private double m_lastVelocitySetpoint;

  public Indexer() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIndexerSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIndexerImmediateCurrentLimitAmps);
    m_spark.setIdleMode(IdleMode.kBrake);
  }

  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }

  @Log(name = "Indexer velocity (rotations per second)")
  public double getVelocity() {
    return m_spark.getEncoder().getVelocity() / 60;
  }

  public boolean hasStopped() {
    return this.getVelocity() == 0 && m_lastVelocity > 0;
  }

  public void periodic() {
    m_lastVelocity = this.getVelocity();
  }
}
