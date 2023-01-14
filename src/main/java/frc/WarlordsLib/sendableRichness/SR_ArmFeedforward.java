package frc.WarlordsLib.sendableRichness;

/**
 * The classes in this package are modifications of existing WPILib classes to output more useful
 * telemetry data. The modifications are written by Eli Barnett (Oblarg) and can be found in this
 * pull request: https://github.com/wpilibsuite/allwpilib/pull/3774
 */
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
@SuppressWarnings("MemberName")
public class SR_ArmFeedforward implements Sendable {
  public double kg;
  private double m_angleRadians;
  public final SR_SimpleMotorFeedforward m_simpleFeedforward;

  /**
   * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate
   * units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public SR_ArmFeedforward(double ks, double kg, double kv, double ka) {
    m_simpleFeedforward = new SR_SimpleMotorFeedforward(ks, kv, ka);
    this.kg = kg;
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
   * Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public SR_ArmFeedforward(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }

  /**
   * Returns the most recent angle of the arm, in radians.
   *
   * @return The position of the arm in radians.
   */
  public double getAngleRadians() {
    return m_angleRadians;
  }

  /**
   * Gets the most recent output.
   *
   * @return Most recent output.
   */
  public double getOutput() {
    return m_simpleFeedforward.getOutput() + kg * Math.cos(m_angleRadians);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint.
   * @param currentVelocityRadPerSec The current velocity setpoint.
   * @param nextVelocityRadPerSec The next velocity setpoint.
   * @param dtSeconds The time until the next velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(
      double positionRadians,
      double currentVelocityRadPerSec,
      double nextVelocityRadPerSec,
      double dtSeconds) {
    m_angleRadians = positionRadians;
    return m_simpleFeedforward.calculate(currentVelocityRadPerSec, nextVelocityRadPerSec, dtSeconds)
        + kg * Math.cos(positionRadians);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint.
   * @param velocityRadPerSec The velocity setpoint.
   * @param accelRadPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(
      double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
    return calculate(
        positionRadians, velocityRadPerSec, velocityRadPerSec + 0.02 * accelRadPerSecSquared, 0.02);
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param positionRadians The position (angle) setpoint.
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionRadians, double velocity) {
    return calculate(positionRadians, velocity, 0);
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm.
   * @param acceleration The acceleration of the arm.
   * @return The maximum possible velocity at the given acceleration and angle.
   */
  public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
    // Assume max velocity is positive
    return m_simpleFeedforward.maxAchievableVelocity(maxVoltage, acceleration)
        - Math.cos(angle) * kg / m_simpleFeedforward.kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm.
   * @param acceleration The acceleration of the arm.
   * @return The minimum possible velocity at the given acceleration and angle.
   */
  public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return m_simpleFeedforward.minAchievableVelocity(maxVoltage, acceleration)
        - Math.cos(angle) * kg / m_simpleFeedforward.kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm.
   * @param velocity The velocity of the arm.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
    return m_simpleFeedforward.maxAchievableAcceleration(maxVoltage, velocity)
        - Math.cos(angle) * kg / m_simpleFeedforward.ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm.
   * @param velocity The velocity of the arm.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, angle, velocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_simpleFeedforward.initSendable(builder);
    builder.setSmartDashboardType("ArmFeedforward");
    builder.addDoubleProperty("kG", () -> kg, kg -> this.kg = kg);
    builder.addDoubleProperty("gravityOutput", () -> kg * Math.cos(m_angleRadians), null);
    builder.addDoubleProperty("output", this::getOutput, null);
  }
}
