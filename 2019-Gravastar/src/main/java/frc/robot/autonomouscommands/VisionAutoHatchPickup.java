/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.tools.PID;
import frc.robot.Robot;


public class VisionAutoHatchPickup extends Command {
  private PID alignmentPID;
	private double p = 0.011;
	private double i= 0.000;
  private double d;
  private double leftPower;
  private double rightPower;
  private double distance;
  private boolean connected;
  private double power;
  private AutoHatchGrab autoHatchGrab= new AutoHatchGrab();
  public VisionAutoHatchPickup() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    alignmentPID = new PID(p, i, d);
		alignmentPID.setMaxOutput(0.4);
		alignmentPID.setMinOutput(-0.4);
    alignmentPID.setSetPoint(0);
    RobotMap.drive.setLowGear();
  }
    public double getAlignmentPIDOutput(){
		Robot.visionCamera.updateVision();
		
		if(Timer.getFPGATimestamp()-Robot.visionCamera.lastParseTime>0.25){
			SmartDashboard.putBoolean("hasCameraValues", false);
			//Robot.changeLightColor.changeLedColor(255, 0, 0);
			//Robot.changeLightColor1.changeLedColor(0, 255, 0);

			
			return 0;
		}
		else{
			SmartDashboard.putBoolean("hasCameraValues", true);
			if(RobotMap.mainUltrasonicSensor2.isConnected()){
				//Robot.changeLightColor.changeLedColor(0, 0, 255);
				//Robot.changeLightColor1.changeLedColor(0, 0, 255);

			}
			else{
				//Robot.changeLightColor.changeLedColor(0, 255, 0);
				//Robot.changeLightColor1.changeLedColor(255, 0, 0);

			}
			if(Robot.visionCamera.getAngle()>25){
				return 0;
			}
			alignmentPID.updatePID(Robot.visionCamera.getAngle());
			return alignmentPID.getResult();
		}
	}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //RobotMap.visionRelay1.set(Value.kForward);
    power = 0.35;
    RobotMap.drive.setLowGear();
    RobotConfig.setDriveMotorsBrake();
    connected = RobotMap.mainUltrasonicSensor2.isConnected();
    distance = RobotMap.mainUltrasonicSensor2.getDistance();
    if(distance>=1.5&&connected&&distance<7){
      OI.pilotController.setRumble(RumbleType.kLeftRumble, 0.0);
      power = Math.pow(distance/15,0.8);
        
    
      rightPower =-power+ RobotMap.drive.getAlignmentPIDOutput();
      leftPower =-power-RobotMap.drive.getAlignmentPIDOutput();
    }
    else if(distance<1.5&&connected){
      power = 0.0;
    }
    else{
      rightPower =-power+ RobotMap.drive.getAlignmentPIDOutput();
      leftPower =-power-RobotMap.drive.getAlignmentPIDOutput();
    }
		
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distance<1.5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    autoHatchGrab.start();
    RobotConfig.setDriveMotorsCoast();
    //RobotMap.visionRelay1.set(Value.kReverse);

    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0   );

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
