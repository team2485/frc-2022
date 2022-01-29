package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeColorSpecific extends ConditionalCommand {

  public IntakeColorSpecific(Intake intake, Alliance alliance) {
    super(
        new InstantCommand(() -> intake.setPWM(kIntakePWM)),
        new InstantCommand(() -> intake.setPWM(kOuttakePWM)),
        () -> {
          return intake.getDetectedColor() == null
              || (intake.getDetectedColor() != null
                  && intake
                      .getDetectedColor()
                      .equals(alliance == Alliance.Red ? kRedBallColor : kBlueBallColor));
        });
  }
}
