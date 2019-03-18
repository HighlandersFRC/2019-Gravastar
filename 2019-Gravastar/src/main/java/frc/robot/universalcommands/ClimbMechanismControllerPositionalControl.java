/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;

public class ClimbMechanismControllerPositionalControl extends Command {
  private double f;
  private double p;
  private double i;
  private double d;
  public ClimbMechanismControllerPositionalControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.climbingMechLeadTalon.selectProfileSlot(0, 0);
    RobotMap.climbingMechLeadTalon.config_kF(0, f, 0);
    RobotMap.climbingMechLeadTalon.config_kP(0, p, 0);
    RobotMap.climbingMechLeadTalon.config_kI(0, i, 0);
    RobotMap.climbingMechLeadTalon.config_kD(0, d, 0);
    RobotMap.climbingMechLeadTalon.set(ControlMode.Position, RobotConfig.climbingMechUpPosition);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotMap.climbingMechLeadTalon.getSelectedSensorPosition()-RobotConfig.climbingMechUpPosition<30;
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
