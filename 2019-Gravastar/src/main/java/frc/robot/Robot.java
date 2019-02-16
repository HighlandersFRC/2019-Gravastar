/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomouscommands.AutoSuite;
import frc.robot.autonomouscommands.EnableDriverControl;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.TeleopSuite;
import frc.robot.tools.Odometry;
import frc.robot.universalcommands.ActuateAllHatchPistons;
import frc.robot.universalcommands.StopMotors;
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

	public static TeleopSuite teleopSuite = new TeleopSuite();
	public static AutoSuite autoSuite  = new AutoSuite();
	private RobotConfig robotConfig = new RobotConfig();
	public static StopMotors stopMotors = new StopMotors();
	private String sanatizedString = "nothing";

	private EnableDriverControl enableDriverControl = new EnableDriverControl();
	
	private UsbCamera camera;
	private UsbCamera camera2;
	public static SerialPort jevois1;
	
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
		
		
		try {
		 jevois1= new SerialPort(115200,edu.wpi.first.wpilibj.SerialPort.Port.kUSB);

		} catch (Exception e) {
			//TODO: handle exception
		}
		robotConfig.setStartingConfig();
		// chooser.addOption("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		camera = CameraServer.getInstance().startAutomaticCapture(0);
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
		try {
			String unsanatizedString = jevois1.readString();
			if(unsanatizedString.isBlank()||unsanatizedString.isEmpty()||unsanatizedString.length()<10){
				
			}
			else{
				sanatizedString = unsanatizedString;
			}
			//SmartDashboard.putString("camReadout",sanatizedString );
		} catch (Exception e) {
			//SmartDashboard.putString("camReadout", "nothing");

		}
		
		/*double distance = visionCamera.getDistance();
		double xDelta = Math.cos(angle)*distance;
		double yDelta = Math.sin(2*angle)*distance;
		if(xDelta<6 && xDelta>1&&yDelta<1.5&&yDelta>-1.5){
			SmartDashboard.putBoolean("DriverAssist availiable", true);
		}
		else{
			SmartDashboard.putBoolean("DriverAssist availiable", false);
		}
		*/
		SmartDashboard.putNumber("armPosit",RobotMap.mainArmEncoder.getAngle());
	
		SmartDashboard.putNumber("leftPos",RobotMap.leftMainDrive.getDistance());
		SmartDashboard.putNumber("rightpos",RobotMap.rightMaindrive.getDistance());
		SmartDashboard.putNumber("analog", RobotMap.mainUltrasonicSensor.getDistance());
	


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
		stopMotors.stopAllMotors();
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
		robotConfig.teleopConfig();
		//teleopSuite.startTeleopCommands();
		autoSuite.startAutoCommands();
		//robotConfig.autoConfig();
		//autoOdometry.zero();
		//autoOdometry.start();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		/*if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}*/
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		/*if(OI.pilotController.getTriggerAxis(Hand.kLeft)>0.5&&OI.pilotController.getTriggerAxis(Hand.kRight)>0.5&&OI.pilotController.getStartButton()&&OI.pilotController.getBackButton()){
			enableDriverControl.startDriverControl();
		}*/
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotConfig.teleopConfig();
		teleopSuite.startTeleopCommands();


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
	
		System.out.print(Robot.jevois1.readString()+":");

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
