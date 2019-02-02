/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors.ArmEncoder;
import frc.robot.tools.PID;
import jaci.pathfinder.Pathfinder;

public class ArmPositionController extends Command {
  private double desiredValue;
  private PID armPID;
  private double armkF = 0.1;
  private double p = 0.08;
  private double i;
  private double d;
  private ArmEncoder armEncoder;
  private boolean shouldRun;
  
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
    shouldRun = true;
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
      SmartDashboard.putNumber("result", armPID.getResult()+Math.cos(Pathfinder.d2r(armEncoder.getAngle()))*0.35);
      SmartDashboard.putNumber("desired", armPID.getSetPoint());
      if(Math.abs(OI.operatorController.getRawAxis(1))<0.1){
        RobotMap.armMaster.set(ControlMode.PercentOutput, armPID.getResult()+Math.cos(Pathfinder.d2r(armEncoder.getAngle()))*armkF);
      }
     

  }
  public void disablePID(){
    shouldRun = false;
      
  }
  public void endablePID(){
    shouldRun = true;
  }
  public double getArmAngle(){
    return armEncoder.getAngle();
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
