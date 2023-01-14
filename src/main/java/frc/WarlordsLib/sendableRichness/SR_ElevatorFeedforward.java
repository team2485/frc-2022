package frc.WarlordsLib.sendableRichness;

/**
 * The classes in this package are modifications of existing WPILib classes to output more useful
 * telemetry data. The modifications are written by Eli Barnett (Oblarg) and can be found in this
 * pull request: https://github.com/wpilibsuite/allwpilib/pull/3774
 */
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor acting
 * against the force of gravity directly).
 */
@SuppressWarnings("MemberName")
public class SR_ElevatorFeedforward implements Sendable {
  public final SR_SimpleMotorFeedforward m_simpleFeedforward;
  public double kg;

  /**
   * Creates a new ElevatorFeedforward with the specified gains. Units of the gain values will
   * dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public SR_ElevatorFeedforward(double ks, double kg, double kv, double ka) {
    m_simpleFeedforward = new SR_SimpleMotorFeedforward(ks, kv, ka);
    this.kg = kg;
  }

  /**
   * Creates a new ElevatorFeedforward with the specified gains. Acceleration gain is defaulted to
   * zero. Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public SR_ElevatorFeedforward(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }

  /**
   * Gets the SimpleMotorFeedforward that describes the motor without the effect of gravity.
   *
   * @return The internal SimpleMotorFeedforward.
   */
  public SR_SimpleMotorFeedforward getSimpleFeedforward() {
    return m_simpleFeedforward;
  }

  /**
   * Gets the most recent output.
   *
   * @return Most recent output.
   */
  public double getOutput() {
    return m_simpleFeedforward.getOutput() + kg;
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param currentVelocity The current velocity setpoint.
   * @param nextVelocity The next velocity setpoint.
   * @param dtSeconds The time until the next velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double currentVelocity, double nextVelocity, double dtSeconds) {
    return m_simpleFeedforward.calculate(currentVelocity, nextVelocity, dtSeconds) + kg;
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double velocity, double acceleration) {
    return m_simpleFeedforward.calculate(velocity, acceleration) + kg;
  }
  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The maximum possible velocity at the given acceleration.
   */
  public double maxAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume max velocity is positive
    return m_simpleFeedforward.maxAchievableVelocity(maxVoltage, acceleration)
        - kg / m_simpleFeedforward.kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param acceleration The acceleration of the elevator.
   * @return The minimum possible velocity at the given acceleration.
   */
  public double minAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return m_simpleFeedforward.minAchievableVelocity(maxVoltage, acceleration)
        - kg / m_simpleFeedforward.kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return m_simpleFeedforward.maxAchievableAcceleration(maxVoltage, velocity)
        - kg / m_simpleFeedforward.ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the elevator.
   * @param velocity The velocity of the elevator.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_simpleFeedforward.initSendable(builder);
    builder.setSmartDashboardType("ElevatorFeedforward");
    builder.addDoubleProperty("kG", () -> kg, kg -> this.kg = kg);
    builder.addDoubleProperty("output", () -> getOutput() + kg, null);
  }
}
