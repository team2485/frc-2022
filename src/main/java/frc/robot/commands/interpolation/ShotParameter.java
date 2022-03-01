package frc.robot.commands.interpolation;

import edu.wpi.first.math.MathUtil;

public class ShotParameter {
  // Variables
  public final double hoodAngleRadians;
  public final double shooterSpeedRotationsPerSecond;
  public final double timeOfFlight;

  public ShotParameter(
      double hoodAngleRadians, double shooterSpeedRotationsPerSecond, double timeOfFlight) {
    this.hoodAngleRadians = hoodAngleRadians;
    this.shooterSpeedRotationsPerSecond = shooterSpeedRotationsPerSecond;
    this.timeOfFlight = timeOfFlight;
  }

  public ShotParameter interpolate(ShotParameter end, double t) {
    return new ShotParameter(
        MathUtil.interpolate(hoodAngleRadians, end.hoodAngleRadians, t),
        MathUtil.interpolate(shooterSpeedRotationsPerSecond, end.shooterSpeedRotationsPerSecond, t),
        MathUtil.interpolate(timeOfFlight, end.timeOfFlight, t));
  }
}
