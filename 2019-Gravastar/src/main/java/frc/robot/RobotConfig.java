package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotConfig {
	public static double gearRatio = 0;
    public static double encoderTicsPerShaftRotation = 0;
    public static double encoderTicsPerWheelRotation = gearRatio*encoderTicsPerShaftRotation;
    public static double wheelDiam = 0.0;
	public static double wheelCircum = Math.PI * wheelDiam;
	public static double robotBaseDist;
    public static double openLoopRampRate = 0.0;
	public static double voltageControlMax = 0.0;
	public static double robotMaxAcceleration = 0.0;
	public static double robotMaxVelocity = 0.0;
	public static int driveMotorContinuousCurrentHighGear = 0;
	public static int driveMotorContinuousCurrentLowGear = 0;
	public static int driveMotorContinuousCurrentLowGearAuto = 0;
	public static int driveMotorContinuousCurrentHighGearAuto = 0;
	public static int driveMotorPeakCurrentLowGear = 0;
	public static int driveMotorPeakCurrentHighGear= 0;		
	public static int driveMotorPeakCurrentHighGearAuto = 0;
	public static int driveMotorPeakCurrentLowGearAuto = 0;
	public static int driveMotorPeakCurrentDurationLowGear = 0;
	public static int driveMotorPeakCurrentDurationHighGear = 0;
	public static int driveMotorPeakCurrentDurationLowGearAuto = 0;
	public static int driveMotorPeakCurrentDurationHighGearAuto = 0;
	public static char robotStartPosition;
	public static double driverDeadZone = 0.15;
	public static int timeOut = 4;//Milliseconds
	public RobotConfig() {
		setStartingConfig();
	}
	public void setStartingConfig() {
	 	RobotMap.navx.zeroYaw();
	 	
	 	RobotMap.rightDriveFollowerOne.set(ControlMode.Follower, RobotMap.rightDriveLeadID);
    	RobotMap.leftDriveFollowerOne.set(ControlMode.Follower, RobotMap.leftDriveLeadID);
    	
    	//Invert the right hand side of the drive train
    	RobotMap.rightDriveLead.setInverted(true);
    	RobotMap.rightDriveFollowerOne.setInverted(true);
    	
    	RobotMap.leftDriveLead.setInverted(false);
        RobotMap.leftDriveFollowerOne.setInverted(false);
    	
    	RobotMap.leftDriveLead.setSelectedSensorPosition(0, 0, 0);
		RobotMap.rightDriveLead.setSelectedSensorPosition(0, 0, 0);
		 
    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationLowGear, RobotConfig.timeOut);
    		talon.enableCurrentLimit(true);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMax, 10);
			talon.enableVoltageCompensation(false); 
			talon.configVoltageMeasurementFilter(32, 10);
		}
    	

	}
	public void autoConfig() {
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.enableVoltageCompensation(true);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configOpenloopRamp(0, 0);
		}
		for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentHighGearAuto, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentHighGearAuto, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationHighGearAuto, RobotConfig.timeOut);
    		talon.enableCurrentLimit(true);
		}
		this.setAllMotorsBreak();
		RobotMap.navx.resetDisplacement();
	}
	public void teleopConfig() {
		RobotMap.shifters.set(RobotMap.highGear);
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.enableVoltageCompensation(false);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configOpenloopRamp(openLoopRampRate, RobotConfig.timeOut);
		}
    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
    	}
		this.setAllMotorsBreak();
		RobotMap.navx.resetDisplacement();
	}
	public void disabledConfig() {
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.set(ControlMode.PercentOutput, 0);
		}
	}
	public void setAllMotorsBreak() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
}
