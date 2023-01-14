package frc.robot.commands.interpolation;

import edu.wpi.first.math.MathUtil;

public class ShotParameter {
  // Variables
  public final double shooterSpeedRotationsPerSecond;
  public final double hoodAngleRadians;

  public final double timeOfFlight;

  public ShotParameter(
      double shooterSpeedRotationsPerSecond, double hoodAngleRadians, double timeOfFlight) {
    this.shooterSpeedRotationsPerSecond = shooterSpeedRotationsPerSecond;
    this.hoodAngleRadians = hoodAngleRadians;
    this.timeOfFlight = timeOfFlight;
  }

  public ShotParameter interpolate(ShotParameter end, double t) {
    return new ShotParameter(
        MathUtil.interpolate(shooterSpeedRotationsPerSecond, end.shooterSpeedRotationsPerSecond, t),
        MathUtil.interpolate(hoodAngleRadians, end.hoodAngleRadians, t),
        MathUtil.interpolate(timeOfFlight, end.timeOfFlight, t));
  }
}
