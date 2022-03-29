package frc.robot.subsystems.cargoHandling;

import static frc.robot.Constants.FeederConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class FeedServo extends SubsystemBase implements Loggable {
  private final Servo m_servo = new Servo(kFeederServoPort);

  @Log(name = "Position setpoint")
  private double m_servoPositionSetpoint = 0;

  @Config.ToggleSwitch(name = "Set servo")
  public void engage(boolean engaged) {
    if (engaged) {
      m_servoPositionSetpoint = kServoEngagedPosition;
    } else {
      m_servoPositionSetpoint = kServoDisengagedPosition;
    }
  }

  @Config.NumberSlider(name = "Set servo position", min = 0, max = 1)
  public void set(double position) {
    m_servoPositionSetpoint = position;
  }

  public double getPositionSetpoint() {
    return m_servoPositionSetpoint;
  }

  @Override
  public void periodic() {
    m_servo.set(m_servoPositionSetpoint);
  }
}
