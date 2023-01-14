package frc.WarlordsLib.sendableRichness;

/**
 * The classes in this package are modifications of existing WPILib classes to output more useful
 * telemetry data. The modifications are written by Eli Barnett (Oblarg) and can be found in this
 * pull request: https://github.com/wpilibsuite/allwpilib/pull/3774
 */
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 */
public class SR_ProfiledPIDController implements Sendable {
  private static int instances;

  private SR_PIDController m_controller;
  private double m_minimumInput;
  private double m_maximumInput;
  private SR_TrapezoidProfile.State m_goal = new SR_TrapezoidProfile.State();
  private SR_TrapezoidProfile.State m_setpoint = new SR_TrapezoidProfile.State();

  public final SR_TrapezoidProfile.Constraints m_constraints;

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  @SuppressWarnings("ParameterName")
  public SR_ProfiledPIDController(
      double Kp, double Ki, double Kd, SR_TrapezoidProfile.Constraints constraints) {
    this(Kp, Ki, Kd, constraints, 0.02);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period The period between controller updates in seconds. The default is 0.02 seconds.
   */
  @SuppressWarnings("ParameterName")
  public SR_ProfiledPIDController(
      double Kp, double Ki, double Kd, SR_TrapezoidProfile.Constraints constraints, double period) {
    m_controller = new SR_PIDController(Kp, Ki, Kd, period);
    m_constraints = constraints;
    instances++;
    MathSharedStore.reportUsage(MathUsageId.kController_ProfiledPIDController, instances);
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp Proportional coefficient
   * @param Ki Integral coefficient
   * @param Kd Differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    m_controller.setPID(Kp, Ki, Kd);
  }

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
    m_controller.setP(Kp);
  }

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
    m_controller.setI(Ki);
  }

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
    m_controller.setD(Kd);
  }

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_controller.getP();
  }

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    return m_controller.getI();
  }

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_controller.getD();
  }

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  public double getPeriod() {
    return m_controller.getPeriod();
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired goal state.
   */
  public void setGoal(SR_TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired goal position.
   */
  public void setGoal(double goal) {
    m_goal = new SR_TrapezoidProfile.State(goal, 0);
  }

  /**
   * Gets the goal for the ProfiledPIDController.
   *
   * @return The goal.
   */
  public SR_TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return True if the error is within the tolerance of the error.
   */
  public boolean atGoal() {
    return atSetpoint() && m_goal.equals(m_setpoint);
  }

  /**
   * Set velocity and acceleration constraints for goal.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public void setConstraints(SR_TrapezoidProfile.Constraints constraints) {
    m_constraints.maxVelocity = constraints.maxVelocity;
    m_constraints.maxAcceleration = constraints.maxAcceleration;
  }

  /**
   * Returns the current setpoint of the ProfiledPIDController.
   *
   * @return The current setpoint.
   */
  public SR_TrapezoidProfile.State getSetpoint() {
    return m_setpoint;
  }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return True if the error is within the tolerance of the error.
   */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_controller.enableContinuousInput(minimumInput, maximumInput);
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_controller.disableContinuousInput();
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller output rather than
   * the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_controller.setTolerance(positionTolerance, velocityTolerance);
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_controller.getPositionError();
  }

  /**
   * Returns the change in error per second.
   *
   * @return The change in error per second.
   */
  public double getVelocityError() {
    return m_controller.getVelocityError();
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @return The controller's next output.
   */
  public double calculate(double measurement) {
    if (m_controller.isContinuousInputEnabled()) {
      // Get error which is smallest distance between goal and measurement
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      double goalMinDistance =
          MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
      double setpointMinDistance =
          MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

      // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
      // may be outside the input range after this operation, but that's OK because the controller
      // will still go there and report an error of zero. In other words, the setpoint only needs to
      // be offset from the measurement by the input range modulus; they don't need to be equal.
      m_goal.position = goalMinDistance + measurement;
      m_setpoint.position = setpointMinDistance + measurement;
    }

    var profile = new SR_TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(getPeriod());
    return m_controller.calculate(measurement, m_setpoint.position);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return The controller's next output.
   */
  public double calculate(double measurement, SR_TrapezoidProfile.State goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PIDController.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return The controller's next output.
   */
  public double calculate(double measurement, double goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   * @return The controller's next output.
   */
  public double calculate(
      double measurement,
      SR_TrapezoidProfile.State goal,
      SR_TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculate(measurement, goal);
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measurement The current measured State of the system.
   */
  public void reset(SR_TrapezoidProfile.State measurement) {
    m_controller.reset();
    m_setpoint = measurement;
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system.
   * @param measuredVelocity The current measured velocity of the system.
   */
  public void reset(double measuredPosition, double measuredVelocity) {
    reset(new SR_TrapezoidProfile.State(measuredPosition, measuredVelocity));
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system. The velocity is assumed to
   *     be zero.
   */
  public void reset(double measuredPosition) {
    reset(measuredPosition, 0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty(
        "positionTolerance",
        m_controller::getPositionTolerance,
        (positionTolerance) ->
            m_controller.setTolerance(positionTolerance, m_controller.getVelocityTolerance()));
    builder.addDoubleProperty(
        "velocityTolerance",
        m_controller::getVelocityTolerance,
        (velocityTolerance) ->
            m_controller.setTolerance(m_controller.getPositionTolerance(), velocityTolerance));
    builder.addDoubleProperty("setpoint", m_controller::getSetpoint, m_controller::setSetpoint);
    builder.addDoubleProperty("positionSetpoint", () -> this.getSetpoint().position, null);
    builder.addDoubleProperty("velocitySetpoint", () -> m_setpoint.velocity, null);

    builder.addDoubleProperty("measurement", m_controller::getMeasurement, null);
    builder.addDoubleProperty("positionError", m_controller::getPositionError, null);
    builder.addDoubleProperty("velocityError", m_controller::getVelocityError, null);
    builder.addBooleanProperty("atSetpoint", m_controller::atSetpoint, null);
    builder.setSmartDashboardType("ProfiledPIDController");
    builder.addDoubleProperty(
        "maxVelocity",
        () -> m_constraints.maxVelocity,
        maxVelocity -> m_constraints.maxVelocity = maxVelocity);
    builder.addDoubleProperty(
        "maxAcceleration",
        () -> m_constraints.maxAcceleration,
        maxAcceleration -> m_constraints.maxAcceleration = maxAcceleration);
    builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
    builder.addDoubleProperty("goalVelocity", () -> getGoal().velocity, null);
  }
}
