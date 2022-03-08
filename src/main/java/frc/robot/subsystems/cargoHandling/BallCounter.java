package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.BooleanSupplier;

public class BallCounter extends SubsystemBase implements Loggable {
  private final DigitalInput m_photoSensor = new DigitalInput(kPhotoSensorPort);

  private BooleanSupplier m_shooterHasDipped;

  private boolean m_lastPhotoSensorOutput = false;

  private int m_numCargo = 0;

  public BallCounter(BooleanSupplier shooterHasDipped) {
    m_shooterHasDipped = shooterHasDipped;
  }

  @Log(name = "Photo sensor")
  public boolean getPhotoSensor() {
    return m_photoSensor.get();
  }

  public void setNumCargo(int num) {
    m_numCargo = num;
  }

  @Log(name = "Num cargo")
  public int getNumCargo() {
    return m_numCargo;
  }

  public void periodic() {
    if (m_lastPhotoSensorOutput == true || m_photoSensor.get()) {
      this.setNumCargo(this.getNumCargo() + 1);
    }

    m_lastPhotoSensorOutput = m_photoSensor.get();

    if (m_shooterHasDipped.getAsBoolean()) {
      this.setNumCargo(this.getNumCargo() - 1);
    }
  }
}
