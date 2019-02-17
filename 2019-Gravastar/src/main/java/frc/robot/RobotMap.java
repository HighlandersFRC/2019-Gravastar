/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.autonomouscommands.PathList;
import frc.robot.sensors.ArmEncoder;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import frc.robot.sensors.UltrasonicSensor;
import frc.robot.sensors.VisionCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    //all 0s are placeholder values
	//Name all Talon ID's for Easy Acess
	//TODO must set all Talon IDs
	public static int rightDriveLeadID = 3;
	public static int leftDriveLeadID = 1;
		
	public static int rightDriveFollowerOneID = 4; 
	public static int leftDriveFollowerOneID = 2;

	public static int armMasterID = 5;
	public static int armFollowerID = 6;

	public static int intakeID = 7;
	//Initialize all TalonsSRX
	public static TalonSRX rightDriveLead = new TalonSRX(rightDriveLeadID);
	public static TalonSRX leftDriveLead = new TalonSRX(leftDriveLeadID);
	
	public static TalonSRX rightDriveFollowerOne = new TalonSRX(rightDriveFollowerOneID);
	public static TalonSRX leftDriveFollowerOne = new TalonSRX(leftDriveFollowerOneID);

	public static TalonSRX armMaster = new TalonSRX(armMasterID);
	public static TalonSRX armFollower = new TalonSRX(armFollowerID);
	public static TalonSRX intake = new TalonSRX(intakeID);

	public static AHRS navx;
	public static Navx mainNavx;
	//Initialize all pneumatic Actuators, predefine actuation directions
	public static DoubleSolenoid shifters = new DoubleSolenoid(0,1);
	public static DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kReverse;//TODO directions must be assigned
	public static DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kForward;//TODO directions mut be assinged
		
	public static DoubleSolenoid hatchPiston1 = new DoubleSolenoid(2, 3);
	public static DoubleSolenoid hatchPiston2 = new DoubleSolenoid(4,5);
	public static DoubleSolenoid hatchPiston3 = new DoubleSolenoid(6, 7);
	public static DoubleSolenoid.Value pushOut = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse;
	public static AnalogInput ultraSonic1;
	public static AnalogInput ultraSonic2;

	public static AnalogInput preassureSensor;

	public static DriveEncoder leftMainDrive = new DriveEncoder(leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
	public static DriveEncoder rightMaindrive = new DriveEncoder(rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));
	public static ArmEncoder mainArmEncoder	 = new ArmEncoder(armMaster);

	public static UltrasonicSensor mainUltrasonicSensor1;
	public static UltrasonicSensor mainUltrasonicSensor2; 



	//Array of drive motors to simplify configuration
	public static TalonSRX driveMotors[] = {
			RobotMap.leftDriveLead,
			RobotMap.rightDriveLead,
			RobotMap.leftDriveFollowerOne,
			RobotMap.rightDriveFollowerOne,
	};
	public static TalonSRX driveMotorLeads[] = {
		RobotMap.leftDriveLead,
		RobotMap.rightDriveLead,
	};
	public static TalonSRX allMotors[] = {
		RobotMap.leftDriveLead,
		RobotMap.rightDriveLead,
		RobotMap.leftDriveFollowerOne,
		RobotMap.rightDriveFollowerOne,
		RobotMap.armMaster,
		RobotMap.armFollower,
		RobotMap.intake,
	};
	public static TalonSRX allMotorLeads[] = {
		RobotMap.leftDriveLead,
		RobotMap.rightDriveLead,
		RobotMap.armMaster,
		RobotMap.intake,
	};
	public static TalonSRX armMotors[] = {
		RobotMap.armMaster,
		RobotMap.armFollower,
	};
	public static TalonSRX armMotorLeads[] = {
		RobotMap.armMaster,
	};
	public static DriveBase drive = new DriveBase();
	public static Arm arm = new Arm();
	public static PathList universalPathList = new PathList();
	public RobotMap(){
		try {
			navx = new AHRS(Port.kMXP);
			mainNavx = new Navx(navx);
			
		} catch (Exception e) {
			System.out.println("navxNotPresent");
		}
		try {
			ultraSonic1 = new AnalogInput(3);
			mainUltrasonicSensor1=new UltrasonicSensor(ultraSonic1);

		} catch (Exception e) {
			System.out.println("UltraSonic1NotPresent");
		}
		try {
			ultraSonic2 = new AnalogInput(0);
			mainUltrasonicSensor2= new UltrasonicSensor(ultraSonic1);

		} catch (Exception e) {
			System.out.println("UltraSonic2NotPresent");
		}
		try {
			preassureSensor = new AnalogInput(2);

		} catch (Exception e) {
			System.out.println("Preassure2NotPresent");
		}
	

	}
}

				   
