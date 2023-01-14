package frc.WarlordsLib;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 * A singleton class for dynamically selecting constants based on the robot in use. Reads a file on
 * the roboRIO with an "ID" and uses that value to select from a provided list of constants.
 */
public class IDManager {
  private static volatile IDManager m_instance;

  private boolean m_idRead;
  private int m_acquiredId;

  /**
   * Instantiates IDManager if not already instantiated. Loads file if not already loaded.
   *
   * @return Singleton instance of IDManager
   */
  public static IDManager getInstance(String filepath) {
    if (m_instance == null) {
      synchronized (IDManager.class) {
        if (m_instance == null) {
          m_instance = new IDManager(filepath);
        }
      }
    }
    return m_instance;
  }

  /**
   * Instantiates IDManager if not already instantiated. Does not load file.
   *
   * @return Singleton instance of IDManager
   */
  public static IDManager getInstance() {
    if (m_instance == null) {
      synchronized (IDManager.class) {
        if (m_instance == null) {
          m_instance = new IDManager();
        }
      }
    }
    return m_instance;
  }

  /**
   * Private singleton constructor DOES load file if possible, because otherwise access in static
   * classes like Constants would not be possible.
   */
  private IDManager(String filepath) {
    m_idRead = false;
    this.readIdFile(filepath);
  }

  private IDManager() {
    m_idRead = false;
  }

  /**
   * Run by the filepath-including constructor.
   *
   * @param filepath the location of the file
   */
  private void readIdFile(String filepath) {
    if (!idFileRead()) {
      File idFile = new File(filepath);

      m_acquiredId = 0; // default (should be competition bot)

      if (idFile.exists()) {
        try {

          BufferedReader reader = new BufferedReader(new FileReader(idFile));
          m_acquiredId = Integer.valueOf(reader.readLine());
          reader.close();
          m_idRead = true;
        } catch (IOException e) {
          e.printStackTrace();
        }
      } else {
        DriverStation.reportWarning("IDManager: id file not found, using default", false);
      }
    }
  }

  public boolean idFileRead() {
    return this.m_idRead;
  }

  public <T> T select(T... values) {
    try {
      System.out.println("id: " + m_acquiredId);
      return values[m_acquiredId];
    } catch (ArrayIndexOutOfBoundsException e) {
      DriverStation.reportWarning("ID index out of bounds, defaulting", true);
      return values[0];
    }
  }
}
