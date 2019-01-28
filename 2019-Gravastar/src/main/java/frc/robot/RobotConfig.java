package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotConfig {
	public static double gearRatio = 7.5;
    public static double encoderTicsPerShaftRotation = 4096;
    public static double encoderTicsPerWheelRotation = gearRatio*encoderTicsPerShaftRotation;
    public static double wheelDiam = 7.75;
	public static double wheelCircum = Math.PI * wheelDiam;
	public static double robotBaseDist = 2.0;
    public static double openLoopRampRate = 0.0;
	public static double voltageControlMaxAuto = 11.0;
	public static double voltageControlMaxTeleop = 11.0;
	public static double robotMaxAcceleration = 10.0;
	public static double robotMaxVelocity = 14.5;
	public static int driveMotorContinuousCurrentHighGear = 30;
	public static int driveMotorContinuousCurrentLowGear = 40;
	public static int driveMotorContinuousCurrentHighGearAuto = 30;
	public static int driveMotorContinuousCurrentLowGearAuto = 40;
	public static int driveMotorPeakCurrentHighGear= 30;		
	public static int driveMotorPeakCurrentLowGear = 60;
	public static int driveMotorPeakCurrentHighGearAuto = 20;
	public static int driveMotorPeakCurrentLowGearAuto = 60;
	public static int driveMotorPeakCurrentDurationLowGear = 100;
	public static int driveMotorPeakCurrentDurationHighGear = 0;
	public static int driveMotorPeakCurrentDurationLowGearAuto = 0;
	public static int driveMotorPeakCurrentDurationHighGearAuto = 0;
	public static char robotStartPosition;
	public static int timeOut = 0;//Milliseconds
	public RobotConfig() {
		setStartingConfig();
	}
	public void setStartingConfig() {
		for(TalonSRX talon:RobotMap.allMotors){
			talon.configFactoryDefault();
		}

		 RobotMap.navx.zeroYaw();
		 
	 	
	 	RobotMap.rightDriveFollowerOne.set(ControlMode.Follower, RobotMap.rightDriveLeadID);
		RobotMap.leftDriveFollowerOne.set(ControlMode.Follower, RobotMap.leftDriveLeadID);
		
		RobotMap.armFollower.set(ControlMode.Follower, RobotMap.armMasterID);
    	
    	//Invert the right hand side of the drive train
    	RobotMap.rightDriveLead.setInverted(false);
		RobotMap.rightDriveFollowerOne.setInverted(InvertType.FollowMaster);

    	RobotMap.leftDriveLead.setInverted(true);
        RobotMap.leftDriveFollowerOne.setInverted(InvertType.FollowMaster);
		
		RobotMap.armMaster.setInverted(false);
		RobotMap.armFollower.setInverted(InvertType.FollowMaster);

    	RobotMap.leftDriveLead.setSelectedSensorPosition(0, 0,0);
		RobotMap.rightDriveLead.setSelectedSensorPosition(0, 0, 0);
		
		RobotMap.leftDriveLead.setSensorPhase(false);
		RobotMap.rightDriveLead.setSensorPhase(false);

    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationLowGear, RobotConfig.timeOut);
    		talon.enableCurrentLimit(true);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxAuto);
			talon.enableVoltageCompensation(false); 
			talon.configVoltageMeasurementFilter(32);
		}
	
    	

	}
	public void autoConfig() {
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxAuto);
			talon.configVoltageMeasurementFilter(32);
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
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxTeleop);
			talon.enableVoltageCompensation(false); 
			talon.configVoltageMeasurementFilter(32);
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
