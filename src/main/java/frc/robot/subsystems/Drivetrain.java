package frc.robot.subsystems;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Drivetrain extends SubsystemBase implements Loggable{
    public final SwerveModule m_frontLeftModule;
    public final SwerveModule m_frontRightModule;
    public final SwerveModule m_backLeftModule;
    public final SwerveModule m_backRightModule;

    public Drivetrain() {  

        m_frontLeftModule = new SwerveModule(kFLDriveTalonPort, kFLTurningTalonPort, kFLCANCoderPort, kFLCANCoderZero, "FL");
        m_frontRightModule = new SwerveModule(kFRDriveTalonPort, kFRTurningTalonPort, kFRCANCoderPort, kFRCANCoderZero, "FR");
        m_backLeftModule = new SwerveModule(kBLDriveTalonPort, kBLTurningTalonPort, kBLCANCoderPort, kBLCANCoderZero, "BL");
        m_backRightModule = new SwerveModule(kBRDriveTalonPort, kBRTurningTalonPort, kBRCANCoderPort, kBRCANCoderZero, "BR");
        
        
    }

    @Override
    public void periodic() {
        //m_backRightModule.setSpeedMetersPerSecond(1);
    }

    
}
