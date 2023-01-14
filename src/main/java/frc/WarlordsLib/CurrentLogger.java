package frc.WarlordsLib;

import edu.wpi.first.wpilibj.Timer;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

/**
 * A singleton class for logging supply current for registered motors to a csv onboard the roboRIO.
 * Used to find appropriate values for current limits.
 */
public class CurrentLogger {
  public static interface CurrentLoggable {
    public double getSupplyCurrent();
  }

  private static volatile CurrentLogger m_instance;

  private File m_file;

  private ArrayList<CurrentLoggable> m_toLog;

  /**
   * Instantiates CurrentLogger if not already instantiated.
   *
   * @return Singleton instance of IDManager
   */
  public static CurrentLogger getInstance() {
    if (m_instance == null) {
      synchronized (CurrentLogger.class) {
        if (m_instance == null) {
          m_instance = new CurrentLogger();
        }
      }
    }
    return m_instance;
  }

  private CurrentLogger() {
    m_toLog = new ArrayList<CurrentLoggable>();
  }

  /** @return whether a log folder has been registered (and thus whether a file has been created) */
  public boolean logFolderRegistered() {
    return m_file != null;
  }

  /**
   * Registers a log folder and creates a new file including the current date/time in which to log
   * current values.
   *
   * @param folderPath path of roboRIO folder (if unsure, use "/home/lvuser/currentLogs" or similar)
   */
  public void registerLogFolder(String folderPath) {
    DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm");
    LocalDateTime now = LocalDateTime.now();

    try {
      m_file = new File(folderPath + "/currentLog_" + now.format(dtf) + ".csv");
      FileWriter writer = new FileWriter(m_file);
      writer.write("time");
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Registers a currentLoggable for which the supply current will be written to file when log() is
   * called. Also, writes the name to the csv file as a key.
   *
   * @param c CurrentLoggable to log (intended for motor controllers)
   * @param name The name to use as a csv key
   */
  public void register(CurrentLoggable c, String name) {
    m_toLog.add(c);

    try {
      FileWriter writer = new FileWriter(m_file, true);
      writer.write(", " + name);
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Writes the current supply currents for each registered CurrentLoggable to a csv file, along
   * with the timestamp.
   */
  public void log() {
    if (m_file != null) {
      try {
        FileWriter writer = new FileWriter(m_file, true);
        writer.write("\n" + Timer.getFPGATimestamp());
        writer.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
      for (CurrentLoggable c : m_toLog) {
        try {
          FileWriter writer = new FileWriter(m_file, true);
          writer.write(", " + String.format("%.3f", c.getSupplyCurrent()));
          writer.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
  }
}
