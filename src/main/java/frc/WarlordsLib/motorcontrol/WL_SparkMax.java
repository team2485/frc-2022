package frc.WarlordsLib.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.WarlordsLib.CurrentLogger.CurrentLoggable;
import frc.WarlordsLib.motorcontrol.base.WPI_SparkMax;
import frc.WarlordsLib.sensors.WL_SparkMaxAlternateEncoder;

/** Warlords wrapper for Spark Max with convenience functions. */
public class WL_SparkMax extends WPI_SparkMax implements CurrentLoggable {

  /**
   * Create a new SPARK MAX Controller
   *
   * @param deviceID The device ID.
   * @param type The motor type connected to the controller. Brushless motors must be connected to
   *     their matching color and the hall sensor
   */
  public WL_SparkMax(int deviceID, MotorType type) {
    super(deviceID, type);
    this.setIdleMode(IdleMode.kCoast);
    this.restoreFactoryDefaults();
    this.clearFaults();
  }

  /**
   * Create a new Brushless SPARK MAX Controller
   *
   * @param deviceID The device ID.
   */
  public WL_SparkMax(int deviceID) {
    this(deviceID, MotorType.kBrushless);
    this.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Sets other sparks to follow this Spark.
   *
   * @param follower the follower motor
   * @param followers any number of follower motors
   */
  public void setFollowers(WL_SparkMax follower, WL_SparkMax... followers) {
    follower.follow(this);
    for (CANSparkMax m : followers) {
      m.follow(this);
    }
  }

  public WL_SparkMaxAlternateEncoder getWLAlternateEncoder(int pulsesPerRevolution) {
    return new WL_SparkMaxAlternateEncoder(this, pulsesPerRevolution);
  }

  public double getSupplyCurrent() {
    return this.get() * this.getOutputCurrent();
  }
}
