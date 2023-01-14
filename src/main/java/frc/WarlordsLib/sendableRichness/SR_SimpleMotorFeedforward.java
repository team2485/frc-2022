package frc.WarlordsLib.sendableRichness;

/**
 * The classes in this package are modifications of existing WPILib classes to output more useful
 * telemetry data. The modifications are written by Eli Barnett (Oblarg) and can be found in this
 * pull request: https://github.com/wpilibsuite/allwpilib/pull/3774
 */
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** A helper class that computes feedforward outputs for a simple permanent-magnet DC motor. */
@SuppressWarnings("MemberName")
public class SR_SimpleMotorFeedforward implements Sendable {
  public double ks;
  public double kv;
  public double ka;

  private double m_velocity;
  private double m_acceleration;
  private double m_output;

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains. Units of the gain values will
   * dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public SR_SimpleMotorFeedforward(double ks, double kv, double ka) {
    this.ks = ks;
    this.kv = kv;
    this.ka = ka;
  }

  /**
   * Creates a new SimpleMotorFeedforward with the specified gains. Acceleration gain is defaulted
   * to zero. Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kv The velocity gain.
   */
  public SR_SimpleMotorFeedforward(double ks, double kv) {
    this(ks, kv, 0);
  }

  /**
   * Gets the velocity corresponding the currently-calculated feedforward output.
   *
   * @return Most recent velocity.
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * Gets the acceleration corresponding the currently-calculated feedforward output.
   *
   * @return Most recent acceleration.
   */
  public double getAcceleration() {
    return m_acceleration;
  }

  /**
   * Gets the most recent output.
   *
   * @return Most recent output.
   */
  public double getOutput() {
    return m_output;
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double velocity, double acceleration) {
    return calculate(velocity, velocity + 0.02 * acceleration, 0.02);
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
    m_velocity = currentVelocity;
    m_acceleration = (nextVelocity - currentVelocity) / dtSeconds;

    var plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
    var feedforward = new LinearPlantInversionFeedforward<>(plant, dtSeconds);

    var r = Matrix.mat(Nat.N1(), Nat.N1()).fill(currentVelocity);
    var nextR = Matrix.mat(Nat.N1(), Nat.N1()).fill(nextVelocity);

    m_output = ks * Math.signum(currentVelocity) + feedforward.calculate(r, nextR).get(0, 0);

    return m_output;
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
   * zero).
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double velocity) {
    return calculate(velocity, 0);
  }

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The maximum possible velocity at the given acceleration.
   */
  public double maxAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - ks - acceleration * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the acceleration constraint, and this will give you a
   * simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param acceleration The acceleration of the motor.
   * @return The minimum possible velocity at the given acceleration.
   */
  public double minAchievableVelocity(double maxVoltage, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks - acceleration * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
   * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
   * simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the motor.
   * @param velocity The velocity of the motor.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, velocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SimpleMotorFeedforward");
    builder.addDoubleProperty("kS", () -> ks, ks -> this.ks = ks);
    builder.addDoubleProperty("kV", () -> kv, kv -> this.kv = kv);
    builder.addDoubleProperty("kA", () -> ka, ka -> this.ka = ka);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("velocityOutput", () -> getVelocity() * kv, null);
    builder.addDoubleProperty("acceleration", this::getAcceleration, null);
    builder.addDoubleProperty("accelerationOutput", () -> getAcceleration() * ka, null);
    builder.addDoubleProperty("output", this::getOutput, null);
  }
}
