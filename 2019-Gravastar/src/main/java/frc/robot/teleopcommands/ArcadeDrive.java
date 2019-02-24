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
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
	private double deadZone = 0.00;
	private double turn =0;
	private double throttel = 0;
	private double ratio = 0;
	private double sensitivity;
	private double leftPower;
	private double rightPower;
	public ArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(RobotMap.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		throttel = -OI.pilotController.getRawAxis(1); 
		ratio = Math.abs(throttel);
		if(Math.abs(OI.pilotController.getRawAxis(4))>deadZone) {	
			turn = -OI.pilotController.getRawAxis(4);
		}
		else {
			turn = 0;
		}
		if(Math.abs(throttel)>0.1){
			leftPower = (throttel - (sensitivity*turn*ratio));
			rightPower = (throttel + (sensitivity*turn*ratio));
		}
	
		else{
			leftPower = (-turn)*sensitivity;
			rightPower = (turn)*sensitivity; 
		}
		if(OI.pilotController.getRawAxis(3)>0.5) {
			leftPower = throttel +(-turn);
			rightPower= throttel +(turn);
		}
		if(Math.abs(leftPower)>1) {
			leftPower = (leftPower/Math.abs(leftPower));
			rightPower = Math.abs(rightPower/leftPower)*(rightPower/Math.abs(rightPower));
		}
		else if(Math.abs(rightPower)>1) {
			rightPower = (rightPower/Math.abs(rightPower));
			leftPower = Math.abs(leftPower/rightPower)*(leftPower/Math.abs(leftPower));
		}
		RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
		if(OI.pilotController.getBumperPressed(Hand.kLeft)){
			RobotMap.drive.setLowGear();
		}
		else if(OI.pilotController.getBumperPressed(Hand.kRight)) {
			RobotMap.drive.setHighGear();
		}
		if(RobotMap.shifters.get() == RobotMap.highGear) {
				sensitivity =1.0;

		}
		else if(RobotMap.shifters.get() == RobotMap.lowGear) {
				sensitivity =0.5;
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
