package frc.WarlordsLib.robotConfigs;

class ConfigsWrapper implements LoadableConfigs, SavableConfigs {

  private RobotConfigs m_robotConfigs;

  private String m_category;

  protected ConfigsWrapper(String category, RobotConfigs robotConfigsInstance) {
    this.m_category = category;
    this.m_robotConfigs = robotConfigsInstance;
  }

  @Override
  public String getString(String key, String backup) {
    return m_robotConfigs.getString(m_category, key, backup);
  }

  @Override
  public double getDouble(String key, double backup) {
    return m_robotConfigs.getDouble(m_category, key, backup);
  }

  @Override
  public int getInt(String key, int backup) {
    return m_robotConfigs.getInt(m_category, key, backup);
  }

  @Override
  public float getFloat(String key, float backup) {
    return m_robotConfigs.getFloat(m_category, key, backup);
  }

  @Override
  public long getLong(String key, long backup) {
    return m_robotConfigs.getLong(m_category, key, backup);
  }

  @Override
  public boolean getBoolean(String key, boolean backup) {
    return m_robotConfigs.getBoolean(m_category, key, backup);
  }

  @Override
  public void put(String key, String value) {
    m_robotConfigs.put(m_category, key, value);
  }

  @Override
  public void put(String key, double value) {
    m_robotConfigs.put(m_category, key, value);
  }

  @Override
  public void put(String key, float value) {
    m_robotConfigs.put(m_category, key, value);
  }

  @Override
  public void put(String key, int value) {
    m_robotConfigs.put(m_category, key, value);
  }

  @Override
  public void put(String key, long value) {
    m_robotConfigs.put(m_category, key, value);
  }

  @Override
  public void put(String key, boolean value) {
    m_robotConfigs.put(m_category, key, value);
  }
}
