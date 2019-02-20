/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
	public static boolean hasCamera;
	private ChangeLightColor changeLightColor = new ChangeLightColor(0,0, 150, RobotMap.canifier1);
	public static VisionCamera visionCamera;
	public static SerialPort jevois1;
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
		try {
			jevois1 = new SerialPort(115200, Port.kUSB);
			hasCamera = true;
		} catch (Exception e) {
			hasCamera = false;
		}
		if(hasCamera){
			visionCamera= new VisionCamera(Robot.jevois1);
		}
		//camera = CameraServer.getInstance().startAutomaticCapture(0);
		//camera = CameraServer.getInstance().startAutomaticCapture(1);

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
		if(hasCamera){
			visionCamera.updateVision();

			SmartDashboard.putNumber("timeSinceGoodValue", Timer.getFPGATimestamp()-visionCamera.lastParseTime);
			SmartDashboard.putString("visionString", visionCamera.getString());
			SmartDashboard.putNumber("visionAngle", visionCamera.getAngle());
			SmartDashboard.putNumber("visionDistance", visionCamera.getDistance());
			
		}
		SmartDashboard.putNumber("rawArmValue", RobotMap.armMaster.getSelectedSensorPosition());
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
		// if(Math.abs(OI.operatorController.getRawAxis(1))>0.15){
		// 	RobotMap.armMaster.set(ControlMode.PercentOutput, OI.operatorController.getRawAxis(1)*-0.50);
		// 	System.out.println(RobotMap.armMaster.getMotorOutputPercent());
		// 	System.out.println(RobotMap.armMaster.getSensorCollection().isRevLimitSwitchClosed() + "reverse");
		// 	System.out.println(RobotMap.armMaster.getSensorCollection().isFwdLimitSwitchClosed() + "forward");

		// }
		// else{
		// 	RobotMap.armMaster.set(ControlMode.PercentOutput, 0);
		// }
		// if(OI.operatorController.getTriggerAxis(Hand.kLeft)>0.1){
		// 	RobotMap.intake.set(ControlMode.PercentOutput, -1.0);
		// }
		// else if(OI.operatorController.getTriggerAxis(Hand.kRight)>0.1){
		// 	RobotMap.intake.set(ControlMode.PercentOutput, 1.0);
		// }
		// else{
		// 	RobotMap.intake.set(ControlMode.PercentOutput, 0);
		// }
		
		SmartDashboard.putNumber("navxValue", RobotMap.mainNavx.currentYaw());
		Scheduler.getInstance().run();
	}
	@Override
	public void testPeriodic() {
	}
	
}
