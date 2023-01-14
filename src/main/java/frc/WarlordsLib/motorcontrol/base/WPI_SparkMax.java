package frc.WarlordsLib.motorcontrol.base;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Basic wrapper for a CAN SparkMax implementing SpeedController and Sendable. */
public class WPI_SparkMax extends CANSparkMax implements Sendable {

  /**
   * Create a new SPARK MAX Controller
   *
   * @param deviceID The device ID.
   * @param type The motor type connected to the controller. Brushless motors must be connected to
   *     their matching color and the hall sensor plugged in. Brushed motors must be connected to
   *     the Red and
   */
  public WPI_SparkMax(int deviceID, MotorType type) {
    super(deviceID, type);

    HAL.report(FRCNetComm.tResourceType.kResourceType_RevSparkMaxCAN, deviceID + 1);
    SendableRegistry.addLW(this, "Spark Max", deviceID);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Speed Controller");
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }
}
