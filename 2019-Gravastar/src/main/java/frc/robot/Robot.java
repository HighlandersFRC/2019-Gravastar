/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.sql.Driver;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomouscommands.AutoSuite;
import frc.robot.sensors.UltrasonicSensor;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.TeleopSuite;
import frc.robot.tools.Odometry;
import frc.robot.universalcommands.ChangeLightColor;
import frc.robot.universalcommands.StopMotors;
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
	public static double forwardUltraSonicAngle;
	public static double forwardUltraSonicAverage;
	public static double reverseUltraSonicAngle;
	public static double reverseUltraSonicAverage;
	public static boolean hasForwardCamera = false;
	public static boolean hasReverseCamera = false;
	public static boolean forwardDriveAssistAvaliable = false;
	public static boolean reverseDriveAssistAvaliable = false;
	public static ChangeLightColor changeLightColor = new ChangeLightColor(0,0, 150, RobotMap.canifier1);
	public static ChangeLightColor changeLightColor2 = new ChangeLightColor(0,0, 150, RobotMap.canifier2);
	public static VisionCamera visionCamera;
	public static VisionCamera visionCamera2;
	public static SerialPort jevois1;
	public static SerialPort jevois2;
	private int runCounter = 0;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		try {
			jevois1 = new SerialPort(115200, Port.kUSB2);
			if(jevois1.getBytesReceived()>2){
				hasForwardCamera = true;
			}
			else{
				hasForwardCamera = false;
			}
		} catch (Exception e) {
			hasForwardCamera = false;
		}
		try {
			jevois2 = new SerialPort(115200, Port.kUSB1);
			if(jevois2.getBytesReceived()>2){
				hasReverseCamera = true;
			}
			else{
				hasReverseCamera = false;
			}
		} catch (Exception e) {
			hasReverseCamera = false;
		}
		visionCamera= new VisionCamera(Robot.jevois1);
		visionCamera2 = new VisionCamera(Robot.jevois2);
		robotConfig.setStartingConfig();
		camera = CameraServer.getInstance().startAutomaticCapture("VisionCamera1", "/dev/video0");
		camera.setResolution(320, 240);
		camera.setFPS(15);
		camera2 = CameraServer.getInstance().startAutomaticCapture("VisionCamera2", "/dev/video1");
		camera2.setResolution(320, 240);
		camera2.setFPS(15);
		camera3= CameraServer.getInstance().startAutomaticCapture("VisionCamera3", "/dev/video2");
		camera3.setResolution(320, 240);
		camera3.setFPS(15);
		server = CameraServer.getInstance().addSwitchedCamera("driverVisionCameras");
		server.setSource(camera);
		
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

		runCounter++;
		if(runCounter%100==0){
			forwardUltraSonicAngle = Math.toDegrees(Math.atan((RobotMap.mainUltrasonicSensor1.getDistance()-RobotMap.mainUltrasonicSensor2.getDistance())/RobotConfig.forwardUltraSonicDisplacementDistance));
			reverseUltraSonicAngle = Math.toDegrees(Math.atan((RobotMap.mainUltrasonicSensor3.getDistance()-RobotMap.mainUltrasonicSensor4.getDistance())/RobotConfig.forwardUltraSonicDisplacementDistance));
			double pressure = ((250*RobotMap.preassureSensor.getAverageVoltage())/4.53)-25;
			SmartDashboard.putNumber("pressure", pressure);
			SmartDashboard.putBoolean("hasNavx", RobotMap.navx.isConnected());
			SmartDashboard.putNumber("leftPos",RobotMap.leftMainDrive.getDistance());
			SmartDashboard.putNumber("rightpos",RobotMap.rightMaindrive.getDistance());
			SmartDashboard.putBoolean("hasForwardCamera", hasForwardCamera);
			SmartDashboard.putBoolean("hasReverseCamera", hasReverseCamera);
			SmartDashboard.putNumber("ultraSonic1",RobotMap.mainUltrasonicSensor1.getDistance());
			SmartDashboard.putNumber("ultraSonic2", RobotMap.mainUltrasonicSensor2.getDistance());
		}
		try {
			if(jevois1.getBytesReceived()<2){
				hasForwardCamera = false;
			}
			else{
				hasForwardCamera = true;
			}
		} catch (Exception e) {
			hasForwardCamera = false;	
		}
		try {
		
			if(jevois1.getBytesReceived()<2){
				hasReverseCamera = false;
			}
			else{
				hasReverseCamera = true;
			}
		} catch (Exception e) {
			hasReverseCamera = false;
		}
		if(OI.operatorController.getPOV() == 180){
			server.setSource(camera3);
		}
		else if(OI.operatorController.getPOV() ==0){
			server.setSource(camera2);
		}
		if(OI.pilotController.getStartButton()){
			RobotMap.visionRelay1.set(Value.kForward);
		}
		else{
			RobotMap.visionRelay1.set(Value.kReverse);
		}
		if(OI.pilotController.getBackButton()){
			RobotMap.visionRelay2.set(Value.kForward);
		}
		else{
			RobotMap.visionRelay2.set(Value.kReverse);
		}
	}
	@Override
	public void disabledInit() {
		teleopSuite.endTeleopCommands();
		autoSuite.endAutoCommands();
		stopMotors.stopAllMotors();
	}

	@Override
	public void disabledPeriodic() {
	

		Scheduler.getInstance().run();
		
	}
	private void visionDecisionAlgorithm(){
		try{
			visionCamera.updateVision();
			forwardUltraSonicAverage = (RobotMap.mainUltrasonicSensor1.getDistance()+RobotMap.mainUltrasonicSensor2.getDistance())/2;
			forwardUltraSonicAngle = Math.toDegrees(Math.atan(RobotConfig.forwardUltraSonicDisplacementDistance/(RobotMap.mainUltrasonicSensor1.getDistance()-RobotMap.mainUltrasonicSensor2.getDistance())));
			if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.25&&RobotMap.mainUltrasonicSensor1.getDistance()>1.5&&RobotMap.mainUltrasonicSensor2.getDistance()>1.5){
				forwardDriveAssistAvaliable = true;
				changeLightColor.changeLedColor(255,0, 0);
			}	
			else if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.5&&RobotMap.mainUltrasonicSensor1.getDistance()>1.5&&RobotMap.mainUltrasonicSensor2.getDistance()>1.5){
				forwardDriveAssistAvaliable = true;
				changeLightColor.changeLedColor(0, 255, 0);
			}	
			else{
				changeLightColor.changeLedColor(0, 0, 255);
				forwardDriveAssistAvaliable = false;
			}	
		}
		catch(Exception e){
		}
		try{
			visionCamera2.updateVision();
			reverseUltraSonicAverage = (RobotMap.mainUltrasonicSensor3.getDistance()+RobotMap.mainUltrasonicSensor4.getDistance())/2;
			reverseUltraSonicAngle = Math.toDegrees(Math.atan(RobotConfig.reverseUltraSonicDisplacementDistance/(RobotMap.mainUltrasonicSensor3.getDistance()-RobotMap.mainUltrasonicSensor4.getDistance())));
			if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.25&&RobotMap.mainUltrasonicSensor3.getDistance()>1.5&&RobotMap.mainUltrasonicSensor4.getDistance()>1.5){
				reverseDriveAssistAvaliable = true;
				changeLightColor.changeLedColor(255,0, 0);
			}	
			else if(Timer.getFPGATimestamp()-visionCamera.lastParseTime<0.5&&RobotMap.mainUltrasonicSensor3.getDistance()>1.5&&RobotMap.mainUltrasonicSensor4.getDistance()>1.5){
				reverseDriveAssistAvaliable = true;
				changeLightColor.changeLedColor(0, 255, 0);
			}	
			else{
				changeLightColor.changeLedColor(0, 0, 255);
				reverseDriveAssistAvaliable = false;
			}	
		}
		catch(Exception e){
		}
	}
	@Override
	public void autonomousInit() {
		robotConfig.autoConfig();
		autoSuite.startAutoCommandsDriverControl();
	}

	@Override
	public void autonomousPeriodic() {
		if(hasForwardCamera){
			visionDecisionAlgorithm();
		}
		else{
			forwardDriveAssistAvaliable = false;
			changeLightColor.changeLedColor(0, 0, 255);
		}
		if(hasReverseCamera){
			visionDecisionAlgorithm();
		}
		else{
			reverseDriveAssistAvaliable = false;
			changeLightColor2.changeLedColor(0, 0, 255);
		}
		if(runCounter%5==0){
			SmartDashboard.putBoolean("forwarddriveAssistAvaliable", forwardDriveAssistAvaliable);
			if(hasForwardCamera){
				SmartDashboard.putString("forwardVisionString", visionCamera.getString());
			}
			SmartDashboard.putBoolean("reverseDriveAssistAvaliable", reverseDriveAssistAvaliable);
			if(hasReverseCamera){
				SmartDashboard.putString("reverseVisionString", visionCamera2.getString());
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
		if(hasForwardCamera){
			visionDecisionAlgorithm();
		}
		else{
			forwardDriveAssistAvaliable = false;
			changeLightColor.changeLedColor(0, 0, 255);
		}
		if(hasReverseCamera){
			visionDecisionAlgorithm();
		}
		else{
			reverseDriveAssistAvaliable = false;
			changeLightColor2.changeLedColor(0, 0, 255);
		}
		if(runCounter%5==0){
			SmartDashboard.putBoolean("forwarddriveAssistAvaliable", forwardDriveAssistAvaliable);
			if(hasForwardCamera){
				SmartDashboard.putString("forwardVisionString", visionCamera.getString());
			}
			SmartDashboard.putBoolean("reverseDriveAssistAvaliable", reverseDriveAssistAvaliable);
			if(hasReverseCamera){
				SmartDashboard.putString("reverseVisionString", visionCamera2.getString());
			}
			SmartDashboard.putNumber("armPosit",RobotMap.mainArmEncoder.getAngle());
		}
		
		Scheduler.getInstance().run();
	}
	@Override
	public void testPeriodic() {
	}
	
}
