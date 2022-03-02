package frc.robot.pit;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PitManager implements Loggable {

  public enum PitState {
    kZeroHood,
    kZeroIntakeArm,
    kZeroClimbElevator,
    kZeroClimbArm;

    @Override
    public String toString() {
      String preProcessedName = super.toString();
      String regex = "([a-z])([A-Z]+)";
      String replace = "$1 $2";
      String titleCase = preProcessedName.substring(1);
      titleCase = titleCase.replaceAll(regex, replace);
      return titleCase.substring(0, 1) + titleCase.toLowerCase().substring(1);
    }
  }

  private PitState m_state;

  @Log(name = "Pit State", tabName = "Pit")
  private SendableChooser<PitState> m_pitStateChooser = new SendableChooser<PitState>();

  public PitManager() {
    m_pitStateChooser.setDefaultOption("Zero Hood", PitState.kZeroHood);
    m_pitStateChooser.addOption("Zero Climb Elevator", PitState.kZeroClimbElevator);
    m_pitStateChooser.addOption("Zero Climb Arm", PitState.kZeroClimbArm);
    m_pitStateChooser.addOption("Zero Intake Arm", PitState.kZeroIntakeArm);

    setState(m_pitStateChooser.getSelected());
  }

  public void setState(PitState state) {
    m_state = state;
  }

  public Trigger getPitStateTrigger(PitState state) {
    return new Trigger(
        () -> {
          return m_state == state;
        });
  }

  public InstantCommand getSetStateCommand(PitState state) {
    return new InstantCommand(() -> this.setState(state));
  }

  public PitState getState() {
    return m_state;
  }

  @Log(name = "Current Climb State", tabName = "Pit")
  public String getStateString() {
    return m_state.toString();
  }
}
