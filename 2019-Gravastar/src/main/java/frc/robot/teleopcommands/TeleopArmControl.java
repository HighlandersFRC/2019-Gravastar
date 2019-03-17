/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.universalcommands.ArmPositionController;
import frc.robot.universalcommands.ClimbMechanismController;
import jaci.pathfinder.Pathfinder;

public class TeleopArmControl extends Command {
  private ArmPositionController armPositionController;
  private ClimbMechanismController climbMechanismController;
  public TeleopArmControl() {
    requires(RobotMap.arm);
   
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.hatchGrabberPiston.set(RobotMap.hatchMechGrab);
    armPositionController = new ArmPositionController(RobotConfig.armUpAngle);
    climbMechanismController = new ClimbMechanismController();
    armPositionController.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(OI.operatorController.getRawAxis(1))>0.25){
      RobotMap.armMaster.set(ControlMode.PercentOutput, OI.operatorController.getRawAxis(1)*-0.60+ Math.cos(Math.toRadians(RobotMap.mainArmEncoder.getAngle()))*RobotConfig.armKfFactor);
      armPositionController.setArmPosition(armPositionController.getArmAngle());
    }
    else if(OI.operatorController.getAButton()){
      armPositionController.setArmPosition(RobotConfig.armRestingAngle);
		}
		else if (OI.operatorController.getYButton()){
			armPositionController.setArmPosition(RobotConfig.armUpAngle);
		}
		else if(OI.operatorController.getXButton()){
		  armPositionController.setArmPosition(70);
		}
		
  
		if(OI.operatorController.getBumper(Hand.kLeft)){
			RobotMap.hatchPushOutPiston.set(RobotMap.hatchMechOut);
		}	
		else{
      RobotMap.hatchPushOutPiston.set(RobotMap.hatchMechIn);
    }
    if(OI.operatorController.getBumper(Hand.kRight)){
      RobotMap.hatchGrabberPiston.set(RobotMap.hatchMechRelease);
    }
    else{
      RobotMap.hatchGrabberPiston.set(RobotMap.hatchMechGrab);
    }

		if(OI.operatorController.getTriggerAxis(Hand.kLeft)>0.1){
      RobotMap.arm.intakeBall();
		}
		else if(OI.operatorController.getTriggerAxis(Hand.kRight)>0.1&&armPositionController.getArmAngle()<80){
      RobotMap.arm.outTakeBall();
    }
    else if(OI.operatorController.getTriggerAxis(Hand.kRight)>0.1&&armPositionController.getArmAngle()>=80){
      RobotMap.arm.shootBall();
		}
		else{
      RobotMap.arm.intakeWheelsResting();
    }
    if(OI.operatorController.getBackButton()&&OI.operatorController.getStartButton()){
      climbMechanismController.start();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
