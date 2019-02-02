/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import frc.robot.tools.DriveTrainVelocityPID;
import frc.robot.tools.PID;

public class DualPIDTapeFollower extends Command {
  private double maxPursueHeight;
  private double minPursureHeight;
  private double maxXPosition;
  private double minXPosition;
  private double heightSetPoint;
  private double xSetPoint;
  private DriveTrainVelocityPID leftVelocityPID;
  private DriveTrainVelocityPID rightVelocityPID;
  private PID yPID;
  private PID xPID;
  private VisionCamera cam;
  private double leftVelocity;
  private double rightVelocity;
  private double xComponent;
  private double yComponent;
  public DualPIDTapeFollower(VisionCamera visionCamera) {
    cam = visionCamera;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftVelocityPID = new DriveTrainVelocityPID(0, RobotMap.leftDriveLead, 1, 0.04430683771, 0.18, 0.0009, 1.80);
    rightVelocityPID = new DriveTrainVelocityPID(0, RobotMap.rightDriveLead, 1, 0.04430683771, 0.18, 0.0009, 1.80);
    yPID = new PID(0, 0, 0);
    xPID = new PID(0,0,0);
    yPID.setMaxOutput(4);
    yPID.setMinOutput(-4);
    xPID.setMaxOutput(4);
    xPID.setMinOutput(-4);
    yPID.setSetPoint(heightSetPoint);
    xPID.setSetPoint(xSetPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
   /* if(cam.getX()<0||cam.getY()<0){
      leftVelocity = 0;
      rightVelocity = 0;
    }
    else{
      xPID.updatePID(cam.getX());
      yPID.updatePID(cam.getY());
      if(cam.getY()<minPursureHeight){
        yComponent = 0;
      }
      else{
        yComponent = yPID.getResult();
      }
      xComponent = xPID.getResult();
      leftVelocity = yComponent-xComponent;
      rightVelocity = yComponent+xComponent;
    } 
    leftVelocityPID.changeDesiredSpeed(leftVelocity);
    rightVelocityPID.changeDesiredSpeed(rightVelocity);*/
    
  
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
