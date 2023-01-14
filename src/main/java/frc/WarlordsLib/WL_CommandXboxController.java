package frc.WarlordsLib;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumMap;

/**
 * An alternative XboxController class used in command-based robot code. Provides JoystickButtons
 * instead of Button values. Using this class to bind a command to a trigger is very simple: i.e.
 * xbox.a().whenPressed(..... Provides JoystickButtons for binding commands to an XboxController's
 * buttons. Additionally offers getters for retrieving axis values.
 */
public class WL_CommandXboxController extends GenericHID {
  private enum XboxPOV {
    kUpper(0),
    kUpperRight(45),
    kRight(90),
    kLowerRight(135),
    kLower(180),
    kLowerLeft(225),
    kLeft(270),
    kUpperLeft(315);

    public final int direction;

    XboxPOV(int direction) {
      this.direction = direction;
    }
  }

  private final EnumMap<Button, JoystickButton> m_buttons =
      new EnumMap<Button, JoystickButton>(Button.class);

  private final EnumMap<XboxPOV, POVButton> m_povButtons =
      new EnumMap<XboxPOV, POVButton>(XboxPOV.class);

  private final XboxController m_hid;
 

  public WL_CommandXboxController(final int port) {
    super(port);
    m_hid = new XboxController(port);


    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  private JoystickButton build(Button button) {
    return new JoystickButton(this, button.value);
  }

  private POVButton build(XboxPOV button) {
    return new POVButton(this, button.direction);
  }

  public JoystickButton leftBumper() {
    return m_buttons.computeIfAbsent(Button.kLeftBumper, this::build);
  }

  public JoystickButton rightBumper() {
    return m_buttons.computeIfAbsent(Button.kRightBumper, this::build);
  }

  public JoystickButton leftStick() {
    return m_buttons.computeIfAbsent(Button.kLeftStick, this::build);
  }

  public JoystickButton rightStick() {
    return m_buttons.computeIfAbsent(Button.kRightStick, this::build);
  }

  public JoystickButton a() {
    return m_buttons.computeIfAbsent(Button.kA, this::build);
  }

  public JoystickButton b() {
    return m_buttons.computeIfAbsent(Button.kB, this::build);
  }

  public JoystickButton x() {
    return m_buttons.computeIfAbsent(Button.kX, this::build);
  }

  public JoystickButton y() {
    return m_buttons.computeIfAbsent(Button.kY, this::build);
  }

  public JoystickButton back() {
    return m_buttons.computeIfAbsent(Button.kBack, this::build);
  }

  public JoystickButton start() {
    return m_buttons.computeIfAbsent(Button.kStart, this::build);
  }

  public POVButton upperPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kUpper, this::build);
  }

  public POVButton upperRightPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kUpperRight, this::build);
  }

  public POVButton rightPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kRight, this::build);
  }

  public POVButton lowerRightPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kLowerRight, this::build);
  }

  public POVButton lowerPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kLower, this::build);
  }

  public POVButton lowerLeftPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kLowerLeft, this::build);
  }

  public POVButton leftPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kLeft, this::build);
  }

  public POVButton upperLeftPOV() {
    return m_povButtons.computeIfAbsent(XboxPOV.kUpperLeft, this::build);
  }
  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  /**
   * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getLeftTriggerAxis() {
    return getRawAxis(Axis.kLeftTrigger.value);
  }

  /**
   * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getRightTriggerAxis() {
    return getRawAxis(Axis.kRightTrigger.value);
  }

  /**
   * Returns AxisButton
   *
   * @param port port number of axis/joystick
   * @param threshold threshold of JoystickAxis
   * @return AxisButton
   */
  public Trigger rightTrigger(double threshold, EventLoop loop) {
    return m_hid.rightTrigger(threshold, loop).castTo(Trigger::new);
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  public Trigger rightTrigger(double threshold) {
    return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger rightTrigger() {
    return rightTrigger(0.5);
  }

  public Trigger leftTrigger(EventLoop loop, double threshold) {
    return m_hid.leftTrigger(threshold, loop).castTo(Trigger::new);
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  public Trigger leftTrigger(double threshold) {
    return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger leftTrigger() {
    return leftTrigger(0.5);
  }


}