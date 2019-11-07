/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArcadeDrive extends Command {
	private double deadZone = 0.00;
	private double turn =0;
	private double throttel = 0;
	private double povValue;
	private double ratio = 0;
	private double sensitivity;
	private double leftPower;
	private double rightPower;
	private double minTurnFactor = 0.4;
	public ArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(RobotMap.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		RobotMap.drive.initAlignmentPID();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double leftPower;
		double rightPower;
		double differential;
		
		throttel = -OI.pilotController.getRawAxis(1); 
		if(throttel ==0){
			throttel = 0.001;
		}
		ratio = Math.abs(1/throttel);
		turn = OI.pilotController.getRawAxis(4);
		differential = (turn*ratio*sensitivity) + Math.abs(minTurnFactor*turn);

		leftPower = (throttel - (differential));
		rightPower = (throttel + (differential));
	
		if(Math.abs(leftPower)>1) {
			rightPower = Math.abs(rightPower/leftPower)*Math.signum(rightPower);
			leftPower = Math.signum(leftPower);
		}
		else if(Math.abs(rightPower)>1) {
			leftPower = Math.abs(leftPower/rightPower)*Math.signum(leftPower);
			rightPower = Math.signum(rightPower);
		}
    	RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
    	RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
		if(OI.pilotController.getBumper(Hand.kRight)){
			RobotMap.drive.setLowGear();
		}
		else if(OI.pilotController.getBumper(Hand.kLeft)) {
			RobotMap.drive.setHighGear();
		}
		if(RobotMap.shifters.get() == RobotMap.highGear) {
				sensitivity =1;
		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =1;
    	}
		if(OI.pilotController.getBButton()){
			RobotMap.visionRelay1.set(Value.kForward);
			double power = 0.35;
			RobotMap.drive.setLowGear();
			RobotConfig.setDriveMotorsBrake();
			boolean connected = RobotMap.mainUltrasonicSensor2.isConnected();
			double distance = RobotMap.mainUltrasonicSensor2.getDistance();
			if(distance>=1.5&&connected&&distance<7){
				OI.pilotController.setRumble(RumbleType.kLeftRumble, 0.0);
				power = Math.pow(distance/15,0.8);
					
			
				rightPower =-power- RobotMap.drive.getAlignmentPIDOutput();
				leftPower =-power+RobotMap.drive.getAlignmentPIDOutput();
			}
			else if(distance<1.5&&connected){
				power = 0.0;
				OI.pilotController.setRumble(RumbleType.kLeftRumble, 0.5);
			}
			else{
				rightPower =-power- RobotMap.drive.getAlignmentPIDOutput();
				leftPower =-power+RobotMap.drive.getAlignmentPIDOutput();
			}
		}
		else{
			RobotMap.visionRelay1.set(Value.kReverse);	
			RobotConfig.setDriveMotorsCoast();
			Robot.changeLightColor.changeLedColor(1, 0, 0);	
			OI.pilotController.setRumble(RumbleType.kLeftRumble, 0.0);
		}

	}

	
	
	 
	

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotState.isDisabled();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.stopMotors.stopDriveTrainMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		this.end();
	}
}
