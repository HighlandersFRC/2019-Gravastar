/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import frc.robot.autonomouscommands.PathList;
import frc.robot.sensors.ArmEncoder;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import frc.robot.sensors.PWMUltraSonicSensor;
import frc.robot.sensors.UltrasonicSensor;
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

	public static AHRS 	navx = new AHRS(Port.kMXP);
	public static Navx mainNavx = new Navx(navx);
	//Initialize all pneumatic Actuators, predefine actuation directions
	public static DoubleSolenoid shifters = new DoubleSolenoid(0,1);
	public static DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kReverse;//TODO directions must be assigned
	public static DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kForward;//TODO directions mut be assinged
		
	public static DoubleSolenoid hatchPiston1 = new DoubleSolenoid(2, 3);
	public static DoubleSolenoid hatchPiston2 = new DoubleSolenoid(4,5);
	public static DoubleSolenoid hatchPiston3 = new DoubleSolenoid(6, 7);
	public static DoubleSolenoid.Value pushOut = DoubleSolenoid.Value.kForward;
	public static DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse;
	public static Counter ultraSonic1 = new Counter(9);
	public static Counter ultraSonic2 = new Counter(8);
	//public static Counter ultraSonic3 = new Counter(7);
	//public static Counter ultraSonic4 = new Counter(6);


	public static AnalogInput preassureSensor = new AnalogInput(1);

	public static DriveEncoder leftMainDrive = new DriveEncoder(leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
	public static DriveEncoder rightMaindrive = new DriveEncoder(rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));
	public static ArmEncoder mainArmEncoder	 = new ArmEncoder(armMaster);
	
	public static PWMUltraSonicSensor mainUltrasonicSensor1=new PWMUltraSonicSensor(ultraSonic1);
	public static PWMUltraSonicSensor mainUltrasonicSensor2= new PWMUltraSonicSensor(ultraSonic2);
	//public static PWMUltraSonicSensor mainUltrasonicSensor3=new PWMUltraSonicSensor(ultraSonic3);
	//public static PWMUltraSonicSensor mainUltrasonicSensor4= new PWMUltraSonicSensor(ultraSonic4);



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
	public static CANifier canifier1 = new CANifier(0);
	public static CANifier canifier2 = new CANifier(0);


	public RobotMap(){
		try {
		
			
		} catch (Exception e) {
		}
		try {

		} catch (Exception e) {
		}
		try {
			

		} catch (Exception e) {
		}
		try {
			

		} catch (Exception e) {
		}
	

	}
}

				   
