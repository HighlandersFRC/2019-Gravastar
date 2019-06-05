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
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
	private VideoSink server;
	public static boolean hasCamera = false;
	private boolean cameraBoolean = false;
	public static ChangeLightColor changeLightColor = new ChangeLightColor(255,0, 0, RobotMap.canifier1);
	public static ChangeLightColor changeLightColor1 = new ChangeLightColor(0,255, 0, RobotMap.canifier2);

	public static VisionCamera visionCamera;
	public static SerialPort jevois1;
	private boolean ableToSwitch;
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
			if(jevois1.getBytesReceived()>2){
				hasCamera = true;
			}
			else{
				hasCamera = true;
			}
		} catch (Exception e) {
			hasCamera = false;
		}
		visionCamera= new VisionCamera(Robot.jevois1);
		ableToSwitch = true;
		
		robotConfig.setStartingConfig();
		camera = CameraServer.getInstance().startAutomaticCapture("VisionCamera1", "/dev/video0");
		camera.setResolution(320, 240);
		camera.setFPS(15);

		camera2 = CameraServer.getInstance().startAutomaticCapture("VisionCamera2", "/dev/video1");
		camera2.setResolution(320, 240);
		camera2.setFPS(15);
		RobotMap.visionRelay1.set(Value.kOn);
	

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
		if(runCounter%10==0){
			visionCamera.updateVision();
			SmartDashboard.putNumber("ultraSonic2", RobotMap.mainUltrasonicSensor2.getDistance());
			SmartDashboard.putNumber("cambytes", jevois1.getBytesReceived());
			SmartDashboard.putString("visionString", visionCamera.getString());
			SmartDashboard.putNumber("visionAngle", visionCamera.getAngle());

		}
		if(runCounter%100==0){

		
				
			
			double pressure = ((250*RobotMap.preassureSensor.getAverageVoltage())/4.53)-25;
			SmartDashboard.putNumber("pressure", pressure);
			SmartDashboard.putBoolean("hasNavx", RobotMap.navx.isConnected());
			SmartDashboard.putNumber("leftPos",RobotMap.leftMainDrive.getDistance());
			SmartDashboard.putNumber("rightpos",RobotMap.rightMaindrive.getDistance());
			SmartDashboard.putBoolean("hasCamera", hasCamera);
			SmartDashboard.putNumber("armPosit", RobotMap.mainArmEncoder.getAngle());
			SmartDashboard.putNumber("ultraSonic1",RobotMap.mainUltrasonicSensor1.getDistance());
			SmartDashboard.putNumber("ultraSonic3",RobotMap.mainUltrasonicSensor3.getDistance());
			SmartDashboard.putNumber("ultraSonic4", RobotMap.mainUltrasonicSensor4.getDistance());
			SmartDashboard.putNumber("lastParse", Timer.getFPGATimestamp()-visionCamera.lastParseTime);
		}
		try{
			if(jevois1.getBytesReceived()>0){
				hasCamera = true;
			}
			else{
				hasCamera = false;
			}
		}
		catch(Exception e){
			hasCamera = false;
		}
		
		// if(OI.pilotController.getStartButton()){
		// 	RobotMap.visionRelay1.set(Value.kForward);
		// }
		// else{
		// 	RobotMap.visionRelay1.set(Value.kReverse);

		// }
	
		
	
		if(OI.pilotController.getTriggerAxis(Hand.kLeft)>0.2&&ableToSwitch){
			if(cameraBoolean){
				server.setSource(camera2);
				cameraBoolean = false;
			}
			else if(!cameraBoolean){
				server.setSource(camera);
				cameraBoolean = true;
			}
			ableToSwitch = false;
		}
		else if(OI.pilotController.getTriggerAxis(Hand.kLeft)<0.2){
			ableToSwitch = true;
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
		
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotConfig.teleopConfig();
		teleopSuite.startTeleopCommands();
	}

	@Override
	public void teleopPeriodic() {	
		Scheduler.getInstance().run();
	}
	@Override
	public void testPeriodic() {
	}
	
}
