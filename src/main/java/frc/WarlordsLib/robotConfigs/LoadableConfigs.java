package frc.WarlordsLib.robotConfigs;

/** Interfaces for getting constant values by type */
public interface LoadableConfigs {

  /**
   * Gets String value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns String value
   */
  String getString(String key, String backup);

  /**
   * Gets double value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns double value
   */
  double getDouble(String key, double backup);

  /**
   * Gets int value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns int value
   */
  int getInt(String key, int backup);

  /**
   * Gets float value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns float value
   */
  float getFloat(String key, float backup);

  /**
   * Gets long value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns long value
   */
  long getLong(String key, long backup);

  /**
   * Gets boolean value from desired constant
   *
   * @param key desired constant
   * @param backup backup value for constant
   * @return returns boolean value
   */
  boolean getBoolean(String key, boolean backup);
}
