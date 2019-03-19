/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.tools.PID;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import edu.wpi.first.wpilibj.Timer;


public class UPIDF extends Command {
    double startTime;
    public UPIDF() {
  }
  // Called just before this Command runs the first time
  @Override
    protected void initialize() {
      RobotMap.climbingMechLeadTalon.setSelectedSensorPosition(RobotConfig.climbingMechStartingPosition);
      RobotMap.climbingMechLeadTalon.config_kP(0, 0.025, 0);
	    RobotMap.climbingMechLeadTalon.config_kI(0, 0.00025, 0);
	    RobotMap.climbingMechLeadTalon.config_kD(0, 0.000025, 0);
      RobotMap.climbingMechLeadTalon.config_kF(0, 0.074066, 0);
      RobotMap.climbingMechLeadTalon.selectProfileSlot(0, 0);
      RobotMap.climbingMechLeadTalon.set(ControlMode.Position, placewherewewanttobe);
      startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(RobotMap.climbingMechLeadTalon.getSelectedSensorPosition()-placewherewewanttobe)<howtrashourcodecanbe){
      return true;
    }
    else{
      return false;
    }
   
    // if (Timer.getFPGATimestamp() - (startTime) >= 2){
      
    //     return true;
    // }
   
    // else{

    //     return false;
    // }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.climbingMechLeadTalon.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
//second try