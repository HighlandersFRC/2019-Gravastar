/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.sensors.ArmEncoder;
import frc.robot.tools.PID;
import jaci.pathfinder.Pathfinder;

public class ArmPositionController extends Command {
  private double desiredValue;
  private PID armPID;
  private double armkF = RobotConfig.armKfFactor;
  private double p = 0.05;
  private double i;
  private double d;
  private ArmEncoder armEncoder;
  private boolean shouldEnd;
  
  public ArmPositionController(double startingAngle) {
    desiredValue = startingAngle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    armEncoder = new ArmEncoder(RobotMap.armMaster);
    armPID = new PID(p, i, d);
    armPID.setMaxOutput(0.4);
    armPID.setMinOutput(-0.4);
    armPID.setSetPoint(desiredValue);
    shouldEnd = false;
  }
  public void setArmPosition(double newValue){
    desiredValue = newValue;
   
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
      
      armPID.setSetPoint(desiredValue);
      if(RobotMap.armMaster.getSensorCollection().isFwdLimitSwitchClosed()){
        armEncoder.setForwardLimitSwitchAngle();
      }
      else if(RobotMap.armMaster.getSensorCollection().isRevLimitSwitchClosed()){
        armEncoder.setReverseLimitSwitchAngle();
      }
      
      armPID.updatePID(armEncoder.getAngle());
      
      if(Math.abs(OI.operatorController.getRawAxis(1))>0.25){
        desiredValue = armEncoder.getAngle();
      }
      else if(armPID.getSetPoint() == RobotConfig.armUpAngle&&armEncoder.getAngle()>80){
        RobotMap.armMaster.set(ControlMode.PercentOutput, 0.2 +Math.cos(Math.toRadians(armEncoder.getAngle()))*RobotConfig.armKfFactor);
      }
      else if(armPID.getSetPoint() == RobotConfig.armRestingAngle&&armEncoder.getAngle()<10){
        RobotMap.armMaster.set(ControlMode.PercentOutput, -0.2);
      }
      else{
        RobotMap.armMaster.set(ControlMode.PercentOutput, armPID.getResult()+Math.cos(Math.toRadians(armEncoder.getAngle()))*RobotConfig.armKfFactor);
      }
     

  }
 
  public double getArmAngle(){
    return armEncoder.getAngle();
  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return shouldEnd;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.stopMotors.stopArmMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
