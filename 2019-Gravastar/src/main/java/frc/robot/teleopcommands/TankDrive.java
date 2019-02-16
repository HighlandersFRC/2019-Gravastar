/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TankDrive extends Command {
	private double deadZone = 0.1;
	public TankDrive() {
		requires(RobotMap.drive);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		if(Math.abs(OI.pilotController.getRawAxis(1))>deadZone){
			RobotMap.leftDriveLead.set(ControlMode.PercentOutput, -OI.pilotController.getRawAxis(1));
		}
		else {
			RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);		
		}
		if(Math.abs(OI.pilotController.getRawAxis(5))>deadZone){
			RobotMap.rightDriveLead.set(ControlMode.PercentOutput, -OI.pilotController.getRawAxis(5));
		}
		else {
			RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);
		}	
		if(OI.pilotController.getBumper(Hand.kRight)) {
			RobotMap.drive.setHighGear();
		}
		else if(OI.pilotController.getBumper(Hand.kLeft)) {
			RobotMap.drive.setLowGear();
		}
	}

	

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (!RobotState.isOperatorControl());
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
