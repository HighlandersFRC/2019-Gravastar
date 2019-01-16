/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;

public class DriveTrainStallProtectionController extends Command {
  public DriveTrainStallProtectionController() {
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
    /*if(RobotMap.shifters.get() == RobotMap.highGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {
  	    talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentHighGear, RobotConfig.timeOut);
  	    talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentHighGear, 0);  
  	    talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationHighGear, 0);
  	    talon.enableCurrentLimit(true);
  	  }
  	}
  	else if(RobotMap.shifters.get() == RobotMap.lowGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {	
  			talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
  	    talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentLowGear, 0);  
  	    talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationLowGear, 0);
  	    talon.enableCurrentLimit(true);
  	  }
  	}*/
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!RobotState.isOperatorControl()){
      return true;
    }
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
