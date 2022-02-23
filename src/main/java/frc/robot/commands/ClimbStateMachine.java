package frc.robot.commands;

import static frc.robot.commands.ClimbCommandBuilder.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbArm;
import frc.robot.subsystems.climb.ClimbElevator;
import java.util.function.BooleanSupplier;

public class ClimbStateMachine {

  private int m_state = 0;

  private enum ClimberState {
    kAligningToMidbar,
    kHookedOnMidbar,
    kClimbingOnMidbar,
    kArmsOnHighBar,
    kMovingMidToHighBar,
    kHookedOnHighbar,
    kPinnedOnHighbar,
    kClimbingOnHighBar,
    kResettingArms,
    kArmsOnTraverseBar,
    kThe22Step
  }

  /**
   * Returns a command based upon the value that m_state currently is. Note: does not change m_state
   * m_state can be set using the {@link #setState(int) setState} method.
   *
   * @see {@link #setState(int) setState}
   */
  public Command runCurrentStep(
      ClimbElevator elevator,
      ClimbArm arm,
      BooleanSupplier proceedButton,
      BooleanSupplier stopButton) {
    ClimberState climberState = ClimberState.values()[m_state];

    switch (climberState) {
      case kAligningToMidbar:
        return lineUpToTape();
        // return new SequentialCommandGroup(lineUpToTape(), getRaiseHookCommand(elevator));
      case kHookedOnMidbar:
        return getHookOnMidBarCommand(elevator); // real
      case kClimbingOnMidbar:
        return backUp();
      case kArmsOnHighBar:
        return hookOnBar();
      case kMovingMidToHighBar:
        return climbVertically();
      case kHookedOnHighbar:
        return swingRackUp();
      case kPinnedOnHighbar:
        return null;
      case kClimbingOnHighBar:
        return null;
      case kResettingArms:
        return null;
      case kArmsOnTraverseBar:
        return null;
      case kThe22Step:
        return getEngageRatchetAndLowerCommand(elevator); // real
      default:
        return null;
    }
  }

  public void setState(int state) {
    m_state = state;
  }

  public int getState() {
    return m_state;
  }
}
