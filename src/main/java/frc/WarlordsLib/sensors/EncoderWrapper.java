package frc.WarlordsLib.sensors;

public interface EncoderWrapper {

  /**
   * Get position of the encoder scaled by setDistancePerRevolution
   *
   * @return position of encoder
   */
  double getPosition();

  /**
   * Reset position of encoder
   *
   * @param position position to set encoder
   */
  void resetPosition(double position);

  /**
   * Set distance per revolution scale factor for encoder
   *
   * @param distance distance per rev for encoder
   */
  void setDistancePerRevolution(double distance);

  /**
   * Get velocity of encoder scaled by setDistancePerRevolution
   *
   * @return velocity of encoder
   */
  double getVelocity();
}
