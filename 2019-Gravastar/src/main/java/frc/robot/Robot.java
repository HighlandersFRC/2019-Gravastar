/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
import frc.robot.universalcommands.ChangeLightColor;
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
	private UsbCamera camera;
	private UsbCamera camera2;
	private UsbCamera camera3;
	private VideoSink server;
	public static boolean hasCamera;
	public static boolean driveAssistAvaliable;
	public static ChangeLightColor changeLightColor = new ChangeLightColor(0,0, 150, RobotMap.canifier1);
	public static VisionCamera visionCamera;
	public static SerialPort jevois1;
	private int runCounter = 0;

	//this odometry is used to provide reference data for the start of paths, it should only be used in autonomous
	public static Odometry autoOdometry = new Odometry(false);
		
	
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		try {
			jevois1 = new SerialPort(115200, Port.kUSB);
			hasCamera = true;
		} catch (Exception e) {
			hasCamera = false;
		}
		if(hasCamera){
			visionCamera= new VisionCamera(Robot.jevois1);
		}
		
		robotConfig.setStartingConfig();
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(320, 240);
		camera.setConnectionStrategy(ConnectionStrategy.kAutoManage);
		camera.setFPS(15);

		camera2= CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setResolution(320, 240);
		camera2.setConnectionStrategy(ConnectionStrategy.kAutoManage);
		camera2.setFPS(15);

		camera3= CameraServer.getInstance().startAutomaticCapture(1);
		camera3.setResolution(320, 240);
		camera3.setConnectionStrategy(ConnectionStrategy.kAutoManage);
		camera3.setFPS(15);

		server = CameraServer.getInstance().getServer();
		Shuffleboard.update();
		SmartDashboard.updateValues(); 

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
		try {
			if(!hasCamera){
				
				jevois1 = new SerialPort(115200, Port.kUSB);
				if(jevois1.getBytesReceived()>2){
					hasCamera = false;
				}
				hasCamera = true;
			}
			
		} catch (Exception e) {
			hasCamera = false;
		}
		runCounter++;
		if(runCounter%100==0){
			double pressure = ((250*RobotMap.preassureSensor.getAverageVoltage())/4.53)-25;
			SmartDashboard.putNumber("pressure", pressure);
			SmartDashboard.putBoolean("hasNavx", RobotMap.navx.isConnected());
			SmartDashboard.putNumber("leftPos",RobotMap.leftMainDrive.getDistance());
			SmartDashboard.putNumber("rightpos",RobotMap.rightMaindrive.getDistance());
			SmartDashboard.putBoolean("hasCamera", hasCamera);
			SmartDashboard.putNumber("ultraSonic1",RobotMap.mainUltrasonicSensor1.getDistance());
			SmartDashboard.putNumber("ultraSonic2", RobotMap.mainUltrasonicSensor2.getDistance());

		}
		if(OI.operatorController.getPOV() == 90){
			server.setSource(camera3);
		}
		else if(OI.operatorController.getPOV() ==0){
			server.setSource(camera);
		}
	}
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
	@Override
	public void autonomousInit() {
		robotConfig.autoConfig();
		autoSuite.startAutoCommandsRobotControl();
		
	}
	@Override
	public void autonomousPeriodic() {
		if(OI.pilotController.getTriggerAxis(Hand.kLeft)>0.5&&OI.pilotController.getTriggerAxis(Hand.kRight)>0.5&&OI.pilotController.getStartButton()&&OI.pilotController.getBackButton()){
			autoSuite.startAutoCommandsDriverControl();
		}
		if(hasCamera){
			visionCamera.updateVision();
			if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.25&&RobotMap.mainUltrasonicSensor1.getDistance()>1&&RobotMap.mainUltrasonicSensor2.getDistance()>1){
				driveAssistAvaliable = true;
				changeLightColor.changeLedColor(255,0, 0);
			}	
			else if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.5&&RobotMap.mainUltrasonicSensor1.getDistance()>1&&RobotMap.mainUltrasonicSensor2.getDistance()>1){
				driveAssistAvaliable = true;
				changeLightColor.changeLedColor(0, 255, 0);
			}	
			else{
				changeLightColor.changeLedColor(0, 0, 255);

				driveAssistAvaliable = false;
			}		
		}
		if(runCounter%5==0){
			if(hasCamera){
				SmartDashboard.putBoolean("driveAssistAvaliable", driveAssistAvaliable);
				//SmartDashboard.putNumber("visionAngle", visionCamera.getAngle());
				SmartDashboard.putString("visionDistance", visionCamera.getString());
				
			}
			SmartDashboard.putNumber("armPosit",RobotMap.mainArmEncoder.getAngle());
			
		}

		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotConfig.teleopConfig();
		teleopSuite.startTeleopCommands();
	}

	@Override
	public void teleopPeriodic() {	
		if(hasCamera){
			visionCamera.updateVision();
			if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.25&&RobotMap.mainUltrasonicSensor1.getDistance()>1&&RobotMap.mainUltrasonicSensor2.getDistance()>1){
				driveAssistAvaliable = true;
				changeLightColor.changeLedColor(255,0, 0);
			}	
			else if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.5&&RobotMap.mainUltrasonicSensor1.getDistance()>1&&RobotMap.mainUltrasonicSensor2.getDistance()>1){
				driveAssistAvaliable = true;
				changeLightColor.changeLedColor(0, 255, 0);
			}	
			else{
				changeLightColor.changeLedColor(0, 0, 255);

				driveAssistAvaliable = false;
			}		
		}
		if(runCounter%5==0){
			if(hasCamera){
				SmartDashboard.putBoolean("driveAssistAvaliable", driveAssistAvaliable);
				SmartDashboard.putString("visionString", visionCamera.getString());
				
			}
			
			SmartDashboard.putNumber("armPosit",RobotMap.mainArmEncoder.getAngle());
			
		}
		
		
	
		
		Scheduler.getInstance().run();
	}
	@Override
	public void testPeriodic() {
	}
	
}
