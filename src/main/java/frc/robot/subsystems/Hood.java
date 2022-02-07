package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput; // fix deprecation warning
// import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team2485.WarlordsLib.PositionPIDSubsystem;
import frc.team2485.WarlordsLib.VelocityPIDSubsystem;
import frc.team2485.WarlordsLib.control.WL_PIDController;
import frc.team2485.WarlordsLib.math.BufferZone;
import frc.team2485.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;

public class Hood extends SubsystemBase
    implements PositionPIDSubsystem, VelocityPIDSubsystem {
    private WL_SparkMax m_spark;
    private RelativeEncoder m_hoodEncoder;

    private WL_PIDController m_velocityController;
    private WL_PIDController m_positionController;

    private BufferZone m_velocityBuffer;

    private boolean isZeroed = false;

    public Hood(RelativeEncoder hoodEncoder) {
	this.m_spark = new WL_SparkMax(Constants.Hood.SPARK_PORT);
	this.m_spark.enableVoltageCompensation(Constants.kNominalVoltage);

	// m_spark.setInverted(true);

	this.m_spark
	    .getForwardLimitSwitch(
		CANDigitalInput.LimitSwitchPolarity.kNormallyClosed)
	    .enableLimitSwitch(true);
	this.m_spark
	    .getReverseLimitSwitch(
		CANDigitalInput.LimitSwitchPolarity.kNormallyClosed)
	    .enableLimitSwitch(true);

	this.m_spark.getEncoder().setPositionConversionFactor(
	    Constants.Hood.HOOD_LEAD_SCREW_GEAR_RATIO);
	this.m_spark.getEncoder().setVelocityConversionFactor(
	    Constants.Hood.HOOD_LEAD_SCREW_GEAR_RATIO);

	this.m_hoodEncoder = hoodEncoder;
	hoodEncoder.setInverted(true);
	this.m_hoodEncoder.setPositionConversionFactor(
	    Constants.Hood.DISTANCE_PER_REVOLUTION);
	this.m_hoodEncoder.setVelocityConversionFactor(
	    Constants.Hood.DISTANCE_PER_REVOLUTION / 60);

	this.m_velocityController = new WL_PIDController();
	this.m_positionController = new WL_PIDController();

	this.m_positionController.setTolerance(1);

	m_velocityBuffer = new BufferZone(
	    Constants.Hood.HOOD_MIN_VELOCITY, Constants.Hood.HOOD_MAX_VELOCITY,
	    Constants.Hood.HOOD_BOTTOM_POSITION_DEG,
	    Constants.Hood.HOOD_TOP_POSITION_DEG,
	    Constants.Hood.BUFFER_ZONE_SIZE);

	RobotConfigs.getInstance().addConfigurable(
	    Constants.Hood.HOOD_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL,
	    m_velocityController);
	RobotConfigs.getInstance().addConfigurable(
	    Constants.Hood.HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL,
	    m_positionController);

	this.addToShuffleboard();
    }

    public void addToShuffleboard() {
	ShuffleboardTab tab = Shuffleboard.getTab(Constants.Hood.TAB_NAME);
	tab.add(this);
	tab.add(m_spark);
	tab.add("Hood Velocity Ctrl", m_velocityController);
	tab.add("Hood Position Ctrl", m_positionController);
	tab.addNumber("Hood Encoder Velocity", this::getEncoderVelocity);
	tab.addNumber("Hood Encoder Position", this::getEncoderPosition);
	tab.addNumber("Hood Neo Encoder Velocity", this::getEncoderVelocity);
	tab.addNumber("Hood Current", m_spark::getOutputCurrent);
	tab.addBoolean("Hood Zeroed", () -> { return isZeroed; });
    }

    @Override
    public double getEncoderPosition() {
	return m_hoodEncoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
	return m_spark.getEncoder().getVelocity();
    }

    @Override
    public void runPositionPID(double position) {
	runVelocityPID(m_positionController.calculate(
	    this.getEncoderPosition(),
	    MathUtil.clamp(position, Constants.Hood.HOOD_BOTTOM_POSITION_DEG,
			   Constants.Hood.HOOD_TOP_POSITION_DEG)));
    }

    @Override
    public void runVelocityPID(double velocity) {
	this.setPWM(m_velocityController.calculate(
	    this.getEncoderVelocity(),
	    m_velocityBuffer.get(velocity, getEncoderPosition())));
    }

    public boolean atPositionSetpoint() {
	return m_positionController.atSetpoint();
    }

    public boolean atVelocitySetpoint() {
	return m_velocityController.atSetpoint();
    }

    @Override
    public void tunePeriodic(int layer) {
	if (layer == 0) {
	    setPWM(m_velocityController.calculate(this.getEncoderVelocity()));
	} else if (layer == 1) {
	    runVelocityPID(
		m_positionController.calculate(this.getEncoderPosition()));
	}
    }

    @Override
    public void setPWM(double pwm) {
	m_spark.set(pwm);
    }

    @Override
    public void resetPIDs() {
	m_velocityController.reset();
	m_positionController.reset();
    }
}
