/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.autonomouscommands.VisionToGetToTarget;
import frc.robot.teleopcommands.ArcadeDrive;

public class DriveTrainController extends Command {
  private VisionToGetToTarget forwardVisionToGetToTarget;
  private VisionToGetToTarget reverseVisionToGetToTarget;

  private ArcadeDrive arcadeDrive;
  private Boolean run;
  private DoubleSolenoid.Value value = RobotMap.highGear;
  public DriveTrainController() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    arcadeDrive = new ArcadeDrive();
    forwardVisionToGetToTarget = new VisionToGetToTarget(false);
    reverseVisionToGetToTarget = new VisionToGetToTarget(true);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    run = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.forwardDriveAssistAvaliable&&OI.pilotController.getStartButtonPressed()&&!forwardVisionToGetToTarget.isRunning()&&!reverseVisionToGetToTarget.isRunning()){
      value = RobotMap.shifters.get();
      //forwardVisionToGetToTarget.start();
    }
    if(Robot.reverseDriveAssistAvaliable&&OI.pilotController.getStartButtonPressed()&&!forwardVisionToGetToTarget.isRunning()&&!reverseVisionToGetToTarget.isRunning()){
      value = RobotMap.shifters.get();
      //reverseVisionToGetToTarget.start();
    }
    else if(!OI.pilotController.getStartButton()&&!OI.pilotController.getBackButton()){
      if(OI.pilotController.getStartButtonReleased()||OI.pilotController.getBackButtonReleased()){
        RobotMap.shifters.set(value);
      }
      if(!arcadeDrive.isRunning()){
        arcadeDrive.start();
      }
       
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
  }
}
