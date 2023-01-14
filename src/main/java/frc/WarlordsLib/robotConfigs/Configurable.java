package frc.WarlordsLib.robotConfigs;

/** Base interfaces for objects that can be loaded and saved with RobotConfigs. */
public interface Configurable {

  /**
   * Loads configs from RobotConfigs
   *
   * @param configs ConfigsWrapper abstraction that exposes only load methods
   */
  void loadConfigs(LoadableConfigs configs);

  /**
   * Saves configs to RobotConfigs
   *
   * @param configs ConfigsWrapper abstraction that exposes only put methods
   */
  void saveConfigs(SavableConfigs configs);
}
