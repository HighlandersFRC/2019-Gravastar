/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;

public class TeleopArmControl extends Command {

  public TeleopArmControl() {
    requires(RobotMap.arm);
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
  
		if(Math.abs(OI.operatorController.getRawAxis(1))>0.1){
      RobotMap.arm.disableArmControl();
      RobotMap.armMaster.set(ControlMode.PercentOutput, OI.operatorController.getRawAxis(1)*-0.65+ Math.cos(Pathfinder.d2r(RobotMap.mainArmEncoder.getAngle()))*0.35);
      
		}
		else{
      RobotMap.arm.enableArmControl();
		}
		if(OI.operatorController.getAButton()){
			RobotMap.arm.changeArmPosition(0);
		}
		else if (OI.operatorController.getYButton()){
      RobotMap.arm.changeArmPosition(90);
		}
		else if(OI.operatorController.getXButton()){
			RobotMap.arm.changeArmPosition(45);
		}
		
		if(OI.operatorController.getBumper(Hand.kLeft)){
			RobotMap.arm.pushOutHatch();
		}	
		else{
			RobotMap.arm.pullInAllHatchPistons();
    }
    if(OI.operatorController.getTriggerAxis(Hand.kLeft)>0.1){
      RobotMap.arm.intakeBall();
    }
    else if(OI.operatorController.getTriggerAxis(Hand.kRight)>0.1){
      RobotMap.arm.shootBall();
    }
    else{
      RobotMap.arm.intakeWheelsResting();
    }
    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
