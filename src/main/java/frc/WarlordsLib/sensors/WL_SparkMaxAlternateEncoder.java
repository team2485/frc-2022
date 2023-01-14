package frc.WarlordsLib.sensors;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Used to interface with an alternate quadrature encoder plugged into a SparkMax. Not to be used
 * with the Hall Sensor.
 */
public class WL_SparkMaxAlternateEncoder implements EncoderWrapper {

  private RelativeEncoder m_encoder;

  public WL_SparkMaxAlternateEncoder(CANSparkMax spark, int pulsesPerRevolution) {
    m_encoder =
        spark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, pulsesPerRevolution);
  }

  public WL_SparkMaxAlternateEncoder(int deviceId, int pulsesPerRevolution) {
    this(new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless), pulsesPerRevolution);
  }

  /**
   * Reset position
   *
   * @param position position to reset to in distance taking into account conversion factor.
   */
  @Override
  public void resetPosition(double position) {
    handleCANError(m_encoder.setPosition(position));
  }

  /**
   * Set the distance per revolution of the encoder
   *
   * @param distance distance per rev
   */
  @Override
  public void setDistancePerRevolution(double distance) {
    handleCANError(m_encoder.setPositionConversionFactor(distance));
    handleCANError(m_encoder.setVelocityConversionFactor(distance));
  }

  /**
   * Get velocity of the encoder is distance per second
   *
   * @return distance per second
   */
  @Override
  public double getVelocity() {
    return m_encoder.getVelocity() / 60; // from distance per minute to distance per second
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  private void handleCANError(REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportWarning("Spark Max Encoder Error: " + error.toString(), true);
    }
  }
}
