
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;

public class RobotConfig {
	public static double gearRatio =1;
    public static double encoderTicsPerShaftRotation = 4096;
    public static double encoderTicsPerWheelRotation = gearRatio*encoderTicsPerShaftRotation;
    public static double wheelDiam = 7.75;
	public static double wheelCircum = Math.PI * wheelDiam;
	public static double robotBaseDist = 2.0;
    public static double openLoopRampRate = 0.0;
	public static double voltageControlMaxAuto = 11.0;
	public static double voltageControlMaxTeleop = 12.3;
	public static double robotMaxAcceleration = 10.0;
	public static double robotMaxVelocity = 14.5;
	public static double armTicksToAngleConversion=-0.02857;//0.02470588;
	public static double armUpAngle = 90;//105;
	public static double armRestingAngle = 0;
	public static double armKfFactor = 0.0;//0;
	public static double ultraSonicConversionFactor = 0.00427807;
	public static double pwmUltraSonicConversionFactor = 0;
	public static double forwardUltraSonicDisplacementDistance = 0.5;
	public static double reverseUltraSonicDisplacementDistance = 0.5;
	public static double armAngleToTicksConversion =1/armTicksToAngleConversion;
	public static int climbingMechMaxPosition;
	public static int climbingMechStartingPosition;
	public static int climbingMechUpPosition;
	public static int driveMotorContinuousCurrentHighGear = 30;
	public static int driveMotorContinuousCurrentLowGear = 40;
	public static int driveMotorPeakCurrentHighGear= 30;		
	public static int driveMotorPeakCurrentLowGear = 60;
	public static int driveMotorPeakCurrentDurationLowGear = 100;
	public static int driveMotorPeakCurrentDurationHighGear = 0;
	public static int armMotorContinuousCurrent = 20;
	public static int armMotorPeakCurrent = 30;
	public static int armMotorPeakCurrentDuration = 500;
	public static int timeOut = 0;//Milliseconds
	public RobotConfig() {
		setStartingConfig();
	}
	public void setStartingConfig() {
		for(TalonSRX talon:RobotMap.allMotors){
			talon.configFactoryDefault();
		}

		 RobotMap.navx.zeroYaw();
		 
		RobotMap.rightDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		RobotMap.leftDriveLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		RobotMap.armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
		RobotMap.climbingMechLeadTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	 	RobotMap.rightDriveFollowerOne.set(ControlMode.Follower, RobotMap.rightDriveLeadID);
		RobotMap.leftDriveFollowerOne.set(ControlMode.Follower, RobotMap.leftDriveLeadID);
		RobotMap.climbingMechFollowerTalon.set(ControlMode.Follower, RobotMap.climbingMechLeadTalonID);
		
		RobotMap.armFollower.set(ControlMode.Follower, RobotMap.armMasterID);
    	
    	//Invert the right hand side of the drive train
    	RobotMap.rightDriveLead.setInverted(false);
		RobotMap.rightDriveFollowerOne.setInverted(InvertType.FollowMaster);

    	RobotMap.leftDriveLead.setInverted(true);
        RobotMap.leftDriveFollowerOne.setInverted(InvertType.FollowMaster);
		
		RobotMap.armMaster.setInverted(false);
		RobotMap.armFollower.setInverted(InvertType.FollowMaster);

		RobotMap.climbingMechLeadTalon.setInverted(false);
		RobotMap.climbingMechFollowerTalon.setInverted(InvertType.FollowMaster);

		RobotMap.intake.setInverted(false);

    	RobotMap.leftDriveLead.setSelectedSensorPosition(0, 0,0);
		RobotMap.rightDriveLead.setSelectedSensorPosition(0, 0, 0);
        //RobotMap.armMaster.setSelectedSensorPosition((int)(105*RobotConfig.armAngleToTicksConversion));
		RobotMap.armMaster.setSelectedSensorPosition((int)(90*RobotConfig.armAngleToTicksConversion), 0, 0);
		RobotMap.climbingMechLeadTalon.setSelectedSensorPosition(climbingMechStartingPosition);

		RobotMap.climbingMechLeadTalon.configForwardSoftLimitEnable(false);
		RobotMap.climbingMechLeadTalon.configReverseSoftLimitEnable(false);
		RobotMap.climbingMechLeadTalon.configForwardSoftLimitThreshold(climbingMechMaxPosition);
		RobotMap.climbingMechLeadTalon.configReverseSoftLimitThreshold(climbingMechStartingPosition);

		RobotMap.leftDriveLead.setSensorPhase(false);
		RobotMap.rightDriveLead.setSensorPhase(false);
		RobotMap.armMaster.setSensorPhase(true);//onCompTrue

    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentHighGear, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentHighGear, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationHighGear, RobotConfig.timeOut);
    		talon.enableCurrentLimit(false);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxAuto);
			talon.enableVoltageCompensation(false); 
			talon.configVoltageMeasurementFilter(32);
		}
		for(TalonSRX talon:RobotMap.armMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.armMotorContinuousCurrent, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.armMotorPeakCurrent, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.armMotorPeakCurrentDuration, RobotConfig.timeOut);
    		talon.enableCurrentLimit(true);
		}
		for(TalonSRX talon:RobotMap.armMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxTeleop);
			talon.enableVoltageCompensation(true); 
			talon.configVoltageMeasurementFilter(32);
		}
		setAllMotorsBrake();
	
    	

	}
	public void autoConfig() {
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configOpenloopRamp(0.25);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxAuto);
			talon.configVoltageMeasurementFilter(32);
			talon.enableVoltageCompensation(true); 
		}
		for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentLowGear, RobotConfig.timeOut);
    		talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationLowGear, RobotConfig.timeOut);
    		talon.enableCurrentLimit(true);
		}
		this.setDriveMotorsCoast();
		RobotMap.navx.resetDisplacement();
	}
	public void teleopConfig() {
		RobotMap.shifters.set(RobotMap.highGear);
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configOpenloopRamp(0.25);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configVoltageCompSaturation(RobotConfig.voltageControlMaxTeleop);
			talon.enableVoltageCompensation(true); 
			talon.configVoltageMeasurementFilter(32);
		}
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.configOpenloopRamp(openLoopRampRate, RobotConfig.timeOut);
		}
    	for(TalonSRX talon:RobotMap.driveMotors) {
    		talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
    	}
		this.setDriveMotorsCoast();
		RobotMap.navx.resetDisplacement();
	}
	public void disabledConfig() {
		for(TalonSRX talon:RobotMap.driveMotors){
			talon.set(ControlMode.PercentOutput, 0);
		}
	}
	public static void setAllMotorsBrake() {
		for(TalonSRX talon:RobotMap.allMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
	public static void setDriveMotorsCoast() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Coast);
        }
	}

	public static void setDriveMotorsBrake() {
		for(TalonSRX talon:RobotMap.driveMotors){
            talon.setNeutralMode(NeutralMode.Brake);
        }
	}
}
