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
	private ChangeLightColor changeLightColor = new ChangeLightColor(255,255, 255);
	//public static VisionCamera visionCamera = new VisionCamera(RobotMap.jevois1);
	private boolean hasNavx;
	//this odometry is used to provide reference data for the start of paths, it should only be used in autonomous
	public static Odometry autoOdometry = new Odometry(false);
	//private VisionCamera visionCamera = new VisionCamera(RobotMap.jevois1);
		
	
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		robotConfig.setStartingConfig();
		// chooser.addOption("My Auto", new MyAutoCommand());
		//camera = CameraServer.getInstance().startAutomaticCapture();
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
		//SmartDashboard.putNumber("visionAngle", visionCamera.getAngle());
		//SmartDashboard.putNumber("visionDistance", visionCamera.getDistance());
		SmartDashboard.putNumber("ultraSonic1",RobotMap.mainUltrasonicSensor1.getDistance());
		SmartDashboard.putNumber("ultraSonic2", RobotMap.mainUltrasonicSensor2.getDistance());

	
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
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		robotConfig.teleopConfig();
		teleopSuite.startTeleopCommands();
	}

	@Override
	public void teleopPeriodic() {
	

	//	System.out.println(RobotMap.jevois1.readString());
		SmartDashboard.putNumber("navxValue", RobotMap.mainNavx.currentYaw());
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
