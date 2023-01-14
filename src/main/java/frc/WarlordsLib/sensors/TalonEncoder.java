package frc.WarlordsLib.sensors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import frc.WarlordsLib.robotConfigs.Configurable;
import frc.WarlordsLib.robotConfigs.LoadableConfigs;
import frc.WarlordsLib.robotConfigs.SavableConfigs;

/** A wrapper for interfacing with an encoder plugged into a TalonSRX or TalonFX. */
public class TalonEncoder extends SensorCollection implements EncoderWrapper, Configurable {

  public enum TalonEncoderType {
    ABSOLUTE,
    QUADRATURE,
    ANALOG
  }

  private double m_distancePerRevolution = 1;

  private int m_pulsesPerRevolution;

  private TalonEncoderType m_encoderType;

  private double m_offset = 0;

  public TalonEncoder(
      BaseTalon motorController, TalonEncoderType encoderType, int pulsesPerRevolution) {
    super(motorController);
    this.m_pulsesPerRevolution = pulsesPerRevolution;
    this.m_encoderType = encoderType;
  }

  public TalonEncoder(int deviceId, TalonEncoderType encoderType, int pulsesPerRevolution) {
    this(new TalonSRX(deviceId), encoderType, pulsesPerRevolution);
  }

  /**
   * Get position of encoder according to set distance per revolution
   *
   * @return position
   */
  @Override
  public double getPosition() {
    switch (m_encoderType) {
      case ABSOLUTE:
        return m_offset =
            this.getPulseWidthPosition() * m_distancePerRevolution / m_pulsesPerRevolution;
      case QUADRATURE:
        return this.getQuadraturePosition() * m_distancePerRevolution / m_pulsesPerRevolution;
      case ANALOG:
        return this.getAnalogInRaw() * m_distancePerRevolution / m_pulsesPerRevolution;
      default:
        return 0;
    }
  }

  /**
   * Reset encoder position
   *
   * @param position position to set to
   */
  @Override
  public void resetPosition(double position) {
    switch (m_encoderType) {
      case ABSOLUTE:
        reportError(
            this.setPulseWidthPosition(
                (int) (position * m_pulsesPerRevolution / m_distancePerRevolution), 50));
        this.m_offset = position - this.getPulseWidthPosition();
        break;
      case QUADRATURE:
        reportError(
            this.setQuadraturePosition(
                (int) (position * m_pulsesPerRevolution / m_distancePerRevolution), 50));
        break;
      case ANALOG:
        reportError(
            this.setAnalogPosition(
                (int) (position * m_pulsesPerRevolution / m_distancePerRevolution), 50));
        break;
    }
  }

  /**
   * Set scaling factor for encoder
   *
   * @param distance distance per revolution
   */
  @Override
  public void setDistancePerRevolution(double distance) {
    this.m_distancePerRevolution = distance;
  }

  public double getDistancePerRevolution() {
    return this.m_distancePerRevolution;
  }

  public void setPulsesPerRevolution(int pulsesPerRevolution) {
    this.m_pulsesPerRevolution = pulsesPerRevolution;
  }

  public int getPulsesPerRevolution() {
    return this.m_pulsesPerRevolution;
  }

  /**
   * Get velocity in distance per second.
   *
   * @return velocity based on given distance per revolution and CPR.
   */
  @Override
  public double getVelocity() {
    switch (m_encoderType) {
      case ABSOLUTE:
        return (this.getPulseWidthVelocity() * m_distancePerRevolution / m_pulsesPerRevolution)
            * 10;
      case QUADRATURE:
        return (this.getQuadratureVelocity() * m_distancePerRevolution / m_pulsesPerRevolution)
            * 10;
      case ANALOG:
        return (this.getAnalogInVel() * m_distancePerRevolution / m_pulsesPerRevolution) * 10;
      default:
        return 0;
    }
  }

  private void reportError(ErrorCode code) {
    if (code != ErrorCode.OK) {
      DriverStation.reportWarning("TalonSRX Encoder Wrapper Error: " + code.toString(), true);
    }
  }

  /**
   * Loads configs from RobotConfigs
   *
   * @param configs ConfigsWrapper abstraction that exposes only load methods
   */
  @Override
  public void loadConfigs(LoadableConfigs configs) {
    this.m_offset = configs.getDouble("offset", this.m_offset);
  }

  /**
   * Saves configs to RobotConfigs
   *
   * @param configs ConfigsWrapper abstraction that exposes only put methods
   */
  @Override
  public void saveConfigs(SavableConfigs configs) {
    configs.put("offset", this.m_offset);
  }
}
