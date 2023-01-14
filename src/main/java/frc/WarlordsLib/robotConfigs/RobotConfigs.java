package frc.WarlordsLib.robotConfigs;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.*;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Singleton for saving robot configs/constants locally on roborio. (Replaces ConstantsIO)
 *
 * <p>Call me by using RobotConfigs.getInstance()
 */
public class RobotConfigs {

  private static final String DEFAULT_CSV_SEPARATOR = ",";

  private static volatile RobotConfigs m_instance;

  private RobotConfigsMap m_configs;

  private ConfigurableRegistry m_configurableRegistry;

  private boolean m_fileLoaded;

  /**
   * Instantiates RobotConfigs singleton. Does NOT load configs from file; please use {@link
   * #loadConfigsFromFile(String)}
   *
   * @return Singleton instance of RobotConfigs
   */
  public static RobotConfigs getInstance() {
    if (m_instance == null) {
      synchronized (RobotConfigs.class) {
        if (m_instance == null) {
          m_instance = new RobotConfigs();
        }
      }
    }
    return m_instance;
  }

  /** Private singleton constructor */
  private RobotConfigs() {
    m_configs = new RobotConfigsMap();
    m_configurableRegistry = new ConfigurableRegistry();
    m_fileLoaded = false;
  }

  /**
   * Returns true if RobotConfigs has successfully loaded from a file.
   *
   * @return m_fileLoaded
   */
  public boolean configsLoadedFromFile() {
    return this.m_fileLoaded;
  }

  /**
   * It is recommended to run this BEFORE running RobotContainer so it may access loaded values!
   *
   * @param filepath the location of the file
   */
  public void loadConfigsFromFile(String filepath) {
    loadConfigsFromFile(filepath, DEFAULT_CSV_SEPARATOR);
  }

  /**
   * Loads configs from file with parsing Use when file is not default CSV
   *
   * @param filepath location of file
   * @param separator delimiter used in file
   */
  public void loadConfigsFromFile(String filepath, String separator) {
    m_configs.clear();
    File file = new File(filepath);
    try {
      if (file.createNewFile()) {
        DriverStation.reportWarning(
            "Constants file not found! Creating new file at " + filepath, false);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    try (BufferedReader reader = new BufferedReader(new FileReader(filepath))) {
      String row;
      while ((row = reader.readLine()) != null) {
        String[] data = row.split(separator);
        if (data.length == 3) {
          m_configs.put(data[0].trim(), data[1].trim(), data[2]);
        }
      }
      this.m_fileLoaded = true;
    } catch (IOException e) {
      e.printStackTrace();
    }

    m_configurableRegistry.updateAll();
  }

  /**
   * Saves config to file
   *
   * @param filepath location to save config file
   */
  public void saveConfigsToFile(String filepath) {
    saveConfigsToFile(filepath, DEFAULT_CSV_SEPARATOR);
  }

  /**
   * Saves config to file but with parsing Use when special delimiter wanted in file (not default
   * CSV)
   *
   * @param filepath location to save config file
   * @param separator desired delimiter for data
   */
  public void saveConfigsToFile(String filepath, String separator) {

    m_configurableRegistry.saveAll();

    try (FileWriter writer = new FileWriter(filepath)) {
      for (String category : m_configs.keySet()) {
        for (String key : m_configs.keySet(category)) {
          if (m_configs.containsKey(category, key)) {
            writer.append(category);
            writer.append(separator);
            writer.append(key);
            writer.append(separator);
            writer.append(m_configs.get(category, key));
            writer.append("\n");
          }
        }
      }

      writer.flush();
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Returns String from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public String getString(String category, String key, String backup) {
    checkConfigsLoaded();
    return m_configs.getStringOrBackup(category, key, backup);
  }

  /**
   * Returns double from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public double getDouble(String category, String key, double backup) {
    checkConfigsLoaded();
    return m_configs.getDoubleOrBackup(category, key, backup);
  }

  /**
   * Returns int from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public int getInt(String category, String key, int backup) {
    checkConfigsLoaded();
    return m_configs.getIntOrBackup(category, key, backup);
  }

  /**
   * Returns float from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public float getFloat(String category, String key, float backup) {
    checkConfigsLoaded();
    return m_configs.getFloatOrBackup(category, key, backup);
  }

  /**
   * Returns long from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public long getLong(String category, String key, long backup) {
    checkConfigsLoaded();
    return m_configs.getLongOrBackup(category, key, backup);
  }

  /**
   * Returns boolean from configs
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param backup desired backup value
   */
  public boolean getBoolean(String category, String key, boolean backup) {
    checkConfigsLoaded();
    return m_configs.getBooleanOrBackup(category, key, backup);
  }

  /** Report warning if RobotConfigs has not loaded a file yet. */
  private void checkConfigsLoaded() {
    if (!configsLoadedFromFile()) {
      DriverStation.reportWarning(
          "RobotConfigs has not loaded a file yet, so no constants have been loaded. Make sure to run method loadConfigsFromFile!",
          true);
    }
  }

  /**
   * Sets desired config info for String value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, String value) {
    m_configs.put(category, key, value);
  }

  /**
   * Sets desired config info for double value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, double value) {
    m_configs.put(category, key, Double.toString(value));
  }

  /**
   * Sets desired config info for float value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, float value) {
    m_configs.put(category, key, Float.toString(value));
  }

  /**
   * Sets desired config info for int value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, int value) {
    m_configs.put(category, key, Integer.toString(value));
  }

  /**
   * Sets desired config info for long value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, long value) {
    m_configs.put(category, key, Long.toString(value));
  }

  /**
   * Sets desired config info for boolean value
   *
   * @param category desired subsystem
   * @param key desired constant
   * @param value value for constant
   */
  public void put(String category, String key, boolean value) {
    m_configs.put(category, key, Boolean.toString(value));
  }

  /** Run the saveConfigurable method of the Configurable */
  public void saveConfigurable(String category, Configurable configurable) {
    configurable.saveConfigs(new ConfigsWrapper(category, this));
  }

  /** Runs the loadConfigurable method of the given Configurable. */
  public void loadConfigurable(String category, Configurable configurable) {
    configurable.loadConfigs(new ConfigsWrapper(category, this));
  }

  /**
   * Adds configurable to ConfigurableRegistry
   *
   * @param category category key name
   * @param configurable loaded config
   */
  public void addConfigurable(String category, Configurable configurable) {
    m_configurableRegistry.addConfigurable(category, configurable);
  }

  private class ConfigurableRegistry {

    private WeakHashMap<String, Configurable> _configurables;

    private ConfigurableRegistry() {
      _configurables = new WeakHashMap<>();
    }

    private void addConfigurable(String category, Configurable configurable) {
      configurable.loadConfigs(new ConfigsWrapper(category, RobotConfigs.getInstance()));
      _configurables.put(category, configurable);
    }

    private void updateAll() {
      for (Map.Entry<String, Configurable> entry : _configurables.entrySet()) {
        entry
            .getValue()
            .loadConfigs(new ConfigsWrapper(entry.getKey(), RobotConfigs.getInstance()));
      }
    }

    private void saveAll() {
      for (Map.Entry<String, Configurable> entry : _configurables.entrySet()) {
        entry
            .getValue()
            .saveConfigs(new ConfigsWrapper(entry.getKey(), RobotConfigs.getInstance()));
      }
    }
  }
}
