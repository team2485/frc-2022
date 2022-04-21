package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbStateMachine implements Loggable {

  public enum ClimbState {
    kNotClimbing,
    kPreClimb,
    kAligningToMidbar,
    kCheckpointAlignedToMidBar,
    kMovingHookToMidBar,
    kCheckpointHookedOnMidBar,
    kClimbingOnMidBar,
    kCheckpointLiftedOnMidBar,
    kFinishingClimbOnMidBar,
    kClimbedOnMidBar,
    kMovingArmsToHighBar,
    kCheckpointArmsOnHighBar,
    kMovingToHighBar,
    kCheckpointHookedOnHighBar,
    kFinishingClimbOnHighBar,
    kClimbedOnHighBar,
    kResettingClimberOnHighBar,
    kMovingArmsToTraverseBar,
    kCheckpointArmsOnTraverseBar,
    kMovingToTraverseBar,
    kCheckpointHookedOnTraverseBar,
    kFinishingClimbOnTraverseBar,
    kClimbedOnTraverseBar;

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

  private ClimbState m_state;
  private ClimbState m_preDisableState;

  public ClimbStateMachine() {
    m_state = ClimbState.kNotClimbing;
    m_preDisableState = ClimbState.kNotClimbing;
  }

  public void setState(ClimbState state) {
    m_state = state;
  }

  public void enableClimb() {
    if (m_preDisableState != ClimbState.kNotClimbing) {
      m_state = m_preDisableState;
    } else {
      m_state = ClimbState.kPreClimb;
    }
  }

  public void disableClimb() {
    m_preDisableState = m_state;
    m_state = ClimbState.kNotClimbing;
  }

  public Trigger getClimbStateTrigger(ClimbState state) {
    return new Trigger(
        () -> {
          return m_state == state;
        });
  }

  public InstantCommand getSetStateCommand(ClimbState state) {
    return new InstantCommand(() -> this.setState(state));
  }

  public ClimbState getState() {
    return m_state;
  }

  @Log(
      name = "Climb State",
      tabName = "RobotContainer",
      width = 2,
      height = 2,
      rowIndex = 0,
      columnIndex = 0)
  public String getStateString() {
    return m_state.toString();
  }
}
