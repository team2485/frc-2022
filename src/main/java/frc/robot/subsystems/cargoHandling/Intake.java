package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import io.github.oblarg.oblog.Loggable;

public class Intake extends SubsystemBase implements Loggable {
  private final WL_SparkMax m_spark = new WL_SparkMax(kIntakeSparkPort);

  @Config(name = "Intake Feedforward")
  private final SR_SimpleMotorFeedforward m_intakeFeedforward =
    new SR_SimpleMotorFeedforward m_intakeFeedforward =
      new SR_SimpleMotorFeedforward (
        kSIntakeVolt, kVIntakeVoltsSecondsPerRadian, kAIntakeVoltsSecondsSquaredPerRadian);

  public Intake() {
    m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
    m_spark.setSmartCurrentLimit(kIntakeSmartCurrentLimitAmps);
    m_spark.setSecondaryCurrentLimit(kIntakeImmediateCurrentLimitAmps);
  }

  public void setPercentOutput(double percentOutput) {
    m_spark.set(percentOutput);
  }
}
