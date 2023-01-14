package frc.WarlordsLib.robotConfigs;

/** Abstract methods for settings config values by type */
public interface SavableConfigs {

  /**
   * Sets String value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, String value);

  /**
   * Sets double value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, double value);

  /**
   * Sets float value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, float value);

  /**
   * Sets int value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, int value);

  /**
   * Sets long value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, long value);

  /**
   * Sets boolean value to desired constant
   *
   * @param key desired constant
   * @param value desired value for constant
   */
  void put(String key, boolean value);
}
