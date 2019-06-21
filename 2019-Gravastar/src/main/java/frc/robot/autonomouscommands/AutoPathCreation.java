/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.autonomouscommands.PathSetup;
public class AutoPathCreation extends Command {
  private double startTime;
  private double doubleAverage;
  private double doubleSum;
  private ArrayList<Double> angleList = new ArrayList<Double>(); 
  public AutoPathCreation() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.visionRelay1.set(RobotMap.lightRingOn);
    startTime = Timer.getFPGATimestamp();
    angleList.clear();
    doubleAverage = 0;
    doubleSum = 0;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println( Math.abs(Timer.getFPGATimestamp()-startTime)>=1);
    if(Robot.visionCamera.getAngle()!=-100){
      angleList.add(Robot.visionCamera.getAngle());
    }

    
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Timer.getFPGATimestamp()-startTime)>=1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Done");
    RobotMap.visionRelay1.set(RobotMap.lightRingOff);
    if(angleList.size()>1){
      for( Double i: angleList){
        doubleSum = doubleSum + i;
      }
      doubleAverage = doubleSum/angleList.size();
      System.out.println(doubleAverage);
      SmartDashboard.putNumber("doubleAverage", doubleSum);
    }
    else{
      System.out.println("no Values");
    }
  
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
