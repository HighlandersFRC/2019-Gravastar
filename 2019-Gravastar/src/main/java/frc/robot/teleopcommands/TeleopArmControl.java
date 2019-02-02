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
import frc.robot.universalcommands.ArmPositionController;
import jaci.pathfinder.Pathfinder;

public class TeleopArmControl extends Command {
  private ArmPositionController armPositionController;
  public TeleopArmControl() {
    requires(RobotMap.arm);
    armPositionController = new ArmPositionController(90);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    armPositionController.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(OI.operatorController.getRawAxis(1))>0.15){
      RobotMap.armMaster.set(ControlMode.PercentOutput, OI.operatorController.getRawAxis(1)*-0.65+ Math.cos(Pathfinder.d2r(RobotMap.mainArmEncoder.getAngle()))*0.35);
      armPositionController.disablePID();
      armPositionController.setArmPosition(armPositionController.getArmAngle());
		}
		else{
      armPositionController.endablePID();
      
		}
  
    if(OI.operatorController.getAButton()){
      armPositionController.setArmPosition(0);
		}
		else if (OI.operatorController.getYButton()){
			armPositionController.setArmPosition(90);
		}
		else if(OI.operatorController.getXButton()){
			armPositionController.setArmPosition(82);
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
