package frc.robot.pit;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.cargoHandling.Hood;
import frc.robot.subsystems.cargoHandling.IntakeArm;
import frc.robot.subsystems.climb.ClimbArm;
import frc.robot.subsystems.climb.ClimbElevator;

public class PitCommandBuilder {
    

    public static Command getZeroHoodCommand(Hood hood){

        return new RunCommand(() -> hood.setVoltage(-3), hood)
			.until(hood::getBottomLimitSwitch).andThen(hood::resetAbsolutePosition);

    }

    public static Command getZeroIntakeArmCommand(IntakeArm intakeArm){
		//TODO: make sure methods are accurate

        return new RunCommand(() -> intakeArm.setVoltage(-3), intakeArm)
			.until(intakeArm::getBottomLimitSwitch).andThen(intakeArm::resetAbsolutePosition);
    }

    public static Command getZeroClimbElevatorCommand(ClimbElevator climbElevator){
		//TODO: make sure methods are accurate
        return new RunCommand(() -> climbElevator.setVoltage(-3), climbElevator)
			.until(() -> climbElevator.getStatorCurrentSpike(10)).andThen(climbElevator::resetAbsolutePosition);

    }
    
    public static Command getZeroClimbArmCommand(ClimbArm climbArm){
		//TODO: make sure methods are accurate
        return new RunCommand(() -> )

    }


    
  
}
