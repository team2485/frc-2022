package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.BooleanSupplier;

public class BallCounter extends SubsystemBase implements Loggable {
  private final DigitalInput m_photoSensor = new DigitalInput(kPhotoSensorPort);
  private final Debouncer m_debouncer = new Debouncer(0.07, DebounceType.kBoth);
  private BooleanSupplier m_shooterHasDipped;

  private boolean m_lastBallDetected = false;

  private int m_numCargo = 0;

  public BallCounter(BooleanSupplier shooterHasDipped) {
    m_shooterHasDipped = shooterHasDipped;
  }

  @Log(name = "Ball detected")
  public boolean getBallDetected() {
    return m_debouncer.calculate(!m_photoSensor.get());
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

  @Config(name = "Reset num cargo")
  public void resetNumCargo(int num) {
    m_numCargo = num;
  }

  public void periodic() {
    if (m_lastBallDetected == true && !this.getBallDetected()) {
      this.setNumCargo(this.getNumCargo() + 1);
    }

    m_lastBallDetected = this.getBallDetected();

    // if (m_shooterHasDipped.getAsBoolean()) {
    //   this.setNumCargo(this.getNumCargo() - 1);
    // }
  }
}
