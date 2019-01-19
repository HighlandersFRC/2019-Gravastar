/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
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
		public static int rightDriveLeadID = 1;
		public static int leftDriveLeadID = 3;
		
		public static int rightDriveFollowerOneID = 2; 
		public static int leftDriveFollowerOneID = 4;//might be there might not
		//Initialize all TalonsSRX
		public static TalonSRX rightDriveLead = new TalonSRX(rightDriveLeadID);
		public static TalonSRX leftDriveLead = new TalonSRX(leftDriveLeadID);
		
		public static TalonSRX rightDriveFollowerOne = new TalonSRX(rightDriveFollowerOneID);
		public static TalonSRX leftDriveFollowerOne = new TalonSRX(leftDriveFollowerOneID);
    		
		public static AHRS navx = new AHRS(I2C.Port.kMXP);
		public static Navx mainNavx = new Navx(navx);	    
		//Initialize all pneumatic Actuators, predefine actuation directions
		//public static DoubleSolenoid shifters = new DoubleSolenoid(0,1);
		//public static DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kOff;//TODO directions must be assigned
		//public static DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kOff;//TODO directions mut be assinged
		
		public static DriveEncoder leftMainDrive = new DriveEncoder(leftDriveLead,RobotMap.leftDriveLead.getSelectedSensorPosition(0));
		public static DriveEncoder rightMaindrive = new DriveEncoder(rightDriveLead,RobotMap.rightDriveLead.getSelectedSensorPosition(0));		
        	
		//Array of drive motors to simplify configuration
		public static TalonSRX driveMotors[] = {
				RobotMap.leftDriveLead,
				RobotMap.rightDriveLead,
				RobotMap.leftDriveFollowerOne,
				RobotMap.rightDriveFollowerOne,
		};
		public static TalonSRX allMotors[] = {
			RobotMap.leftDriveLead,
			RobotMap.rightDriveLead,
			RobotMap.leftDriveFollowerOne,
			RobotMap.rightDriveFollowerOne,
		};
		public static DriveBase drive = new DriveBase();
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
