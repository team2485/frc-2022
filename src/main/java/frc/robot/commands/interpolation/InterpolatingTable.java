package frc.robot.commands.interpolation;

import static frc.robot.Constants.*;

import java.util.Map.Entry;

// Interpolating table
public class InterpolatingTable {

  /* Private constructor because this is a utility class */
  private InterpolatingTable() {}

  // Method to get shot parameters based on vision distances
  public static ShotParameter get(double distance) {
    Entry<Double, ShotParameter> ceilEntry = kShootingMap.ceilingEntry(distance);
    Entry<Double, ShotParameter> floorEntry = kShootingMap.floorEntry(distance);
    if (ceilEntry == null) return floorEntry.getValue();
    if (floorEntry == null) return ceilEntry.getValue();
    return ceilEntry
        .getValue()
        .interpolate(
            floorEntry.getValue(),
            (distance - floorEntry.getKey()) / (ceilEntry.getKey() - floorEntry.getKey()));
  }
}
