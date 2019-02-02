/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomouscommands.AutoSuite;
import frc.robot.autonomouscommands.CascadingDriveStraightPID;
import frc.robot.sensors.ArmEncoder;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.TeleopSuite;
import frc.robot.tools.DriveTrainVelocityPID;
import frc.robot.tools.Odometry;
import frc.robot.universalcommands.ActuateAllHatchPistons;
import frc.robot.universalcommands.ArmPositionController;
import frc.robot.universalcommands.StopAllMotors;
import jaci.pathfinder.Pathfinder;
//import org.json.simple.parser.JSONParser;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private TeleopSuite teleopSuite = new TeleopSuite();
	private AutoSuite autoSuite  = new AutoSuite();
	private RobotConfig robotConfig = new RobotConfig();
	private StopAllMotors stopAllMotors = new StopAllMotors();
	private ActuateAllHatchPistons actuateAllHatchPistons = new ActuateAllHatchPistons();
	private ArmPositionController armPositionController = new ArmPositionController(90);
	//private VisionCamera visionCamera = new VisionCamera(RobotMap.jevois1);
	
	private UsbCamera camera;
	private UsbCamera camera2;
	
	//this odometry is used to provide reference data for the start of paths, it should only be used in autonomous
	public static Odometry autoOdometry = new Odometry(false);
	//private VisionCamera visionCamera = new VisionCamera(RobotMap.jevois1);
	Command m_autonomousCommand;
	Command autCommand;  
		
	
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		robotConfig.setStartingConfig();
		// chooser.addOption("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		//camera = CameraServer.getInstance().startAutomaticCapture(0);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use
	 * this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		double pressure = ((250*RobotMap.preassureSensor.getAverageVoltage())/4.53)-25;
	
		SmartDashboard.putNumber("pressure", pressure);
		SmartDashboard.putBoolean("hasNavx", RobotMap.navx.isConnected());
		SmartDashboard.putNumber("armPosit",RobotMap.mainArmEncoder.getAngle());
	
		SmartDashboard.putNumber("leftPos",RobotMap.leftMainDrive.getDistance());
		SmartDashboard.putNumber("rightpos",RobotMap.rightMaindrive.getDistance());
		//SmartDashboard.putNumber("Distance",visionCamera.getDist());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		teleopSuite.endTeleopCommands();
		autoSuite.endAutoCommands();
		autoOdometry.cancel();
		autoOdometry.endOdmetry();
		stopAllMotors.start();
	}

	@Override
	public void disabledPeriodic() {
	

		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		autoSuite.startAutoCommands();
		robotConfig.autoConfig();
		autoOdometry.zero();
		autoOdometry.start();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotConfig.teleopConfig();
		teleopSuite.startTeleopCommands();

		armPositionController.start();
		armPositionController.setArmPosition(90);

		//visionCamera.start();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//System.out.println(RobotMap.jevois1.readString());
		/*if(Math.abs(OI.operatorController.getRawAxis(1))>0.1){
			RobotMap.armMaster.set(ControlMode.PercentOutput, OI.operatorController.getRawAxis(1)*-0.65+ Math.cos(Pathfinder.d2r(RobotMap.mainArmEncoder.getAngle()))*0.35);
		}
		else{
			RobotMap.armMaster.set(ControlMode.PercentOutput, Math.cos(Pathfinder.d2r(RobotMap.mainArmEncoder.getAngle()))*0.35);	
		}*/
		if(OI.operatorController.getAButton()){
			armPositionController.setArmPosition(0);
		}
		else if (OI.operatorController.getYButton()){
			armPositionController.setArmPosition(90);
		}
		else if(OI.operatorController.getXButton()){
			armPositionController.setArmPosition(82);
		}
		
		if(OI.operatorController.getBumper(Hand.kLeft)){
			actuateAllHatchPistons.actuatePistons(RobotMap.pushOut);
		}	
		else{
			actuateAllHatchPistons.actuatePistons(RobotMap.in);
		}
		if(OI.operatorController.getTriggerAxis(Hand.kLeft)>0.1){
			RobotMap.intake.set(ControlMode.PercentOutput, -0.4);
		}
		else if(OI.operatorController.getTriggerAxis(Hand.kRight)>0.1){
			RobotMap.intake.set(ControlMode.PercentOutput, 1);
		}
		else{
			RobotMap.intake.set(ControlMode.PercentOutput, 0);

		}
		SmartDashboard.putNumber("navxValue", RobotMap.mainNavx.currentYaw());
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
