package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable{
  private final WPI_TalonSRX m_talon = new WPI_TalonSRX(kIntakePort);

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(kI2CPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public Intake() {
    m_colorMatcher.addColorMatch(kBlueBallColor);
    m_colorMatcher.addColorMatch(kRedBallColor);
  }

  @Log(name = "Detected Color")
  public String getDetectedColorString() {
    String colorString;
    if(this.getDetectedColor() == kBlueBallColor) {
      colorString = "Blue";
    } else if (this.getDetectedColor() == kRedBallColor) {
      colorString = "Red";
    } else {
      colorString = "Unknown, RGB:" + this.getDetectedColor().red + ", " + this.getDetectedColor().green + ", " + this.getDetectedColor().green  ;
    }

    return colorString;
  }

  @Log(name = "Detected Color Confidence")
  public double getDetectedColorConfidence() {
    return this.getDetectedColorResult().confidence;
  }

  @Log(name = "Detected Proximity")
  public double getProximity() {
    return m_colorSensor.getProximity();
  }

  public Color getDetectedColor() {
    return getDetectedColorResult().color;
  }
  public ColorMatchResult getDetectedColorResult() {
    return m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
  }

  public void setVoltage(double voltage) {
    m_talon.setVoltage(voltage);
  }

  public void setPWM (double pwm) {
    m_talon.set(ControlMode.PercentOutput, pwm);
  }
}
