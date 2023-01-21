package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class TargetVision extends SubsystemBase {
    private PhotonCamera m_camera;
    private double yawVal = 0, pitchVal = 0, skewVal = 0, areaVal = 0;
    private boolean hasTarget = false;
    private boolean LED_Enable = false;

    public TargetVision() {
        this.m_camera = new PhotonCamera(VisionConstants.kCameraName);
        this.m_camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        var result = this.m_camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget = true;

            SmartDashboard.putNumber("Yaw Value", yawVal);
            SmartDashboard.putNumber("Pitch Value", pitchVal);
            SmartDashboard.putNumber("Skew Value", skewVal);
            SmartDashboard.putNumber("Area Value", areaVal);
            SmartDashboard.putBoolean("LED On/Off", hasTarget);
        } else {
            this.hasTarget = false;
        }

        if (LED_Enable) {
            cameraLEDOn();
            m_camera.setDriverMode(true);
        } else {
            cameraLEDOff();
            m_camera.setDriverMode(false);
        }
    }

    @Override
    public void simulationPeriodic() {

    }

    public double getYawVal() {
        return this.yawVal;
    }

    public double getPitchVal() {
        return this.pitchVal;
    }

    public double getSkewVal() {
        return this.skewVal;
    }

    public double getAreaVal() {
        return this.areaVal;
    }

    public boolean hasTarget() {
        return this.hasTarget;
    }

    public void cameraLEDOn() {
        this.m_camera.setLED(VisionLEDMode.kOn);
    }

    public void cameraLEDOff() {
        this.m_camera.setLED(VisionLEDMode.kOff);
    }

    public void cameraLEDBlink() {
        this.m_camera.setLED(VisionLEDMode.kBlink);
    }

    public void cameraLEDToggle() {
        if (LED_Enable) LED_Enable = false;
        else LED_Enable = true;
    }

    public void cameraLED() {
        this.m_camera.setLED(VisionLEDMode.kDefault);
    }

    public double getRange() {
        double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kLensHeightMeters, VisionConstants.kTargetHeightMeters, VisionConstants.kLensPitchRadians, Units.degreesToRadians(getPitchVal()));
        SmartDashboard.putNumber("Camera Distance", range);
        return range;
    }
}
