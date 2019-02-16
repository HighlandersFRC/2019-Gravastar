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
import frc.robot.sensors.VisionCamera;
import jaci.pathfinder.Pathfinder;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;

public class VisionToGetToTarget extends Command {
  private Notifier camNotifier;
  private ArrayList<Double> xDeltaArrayList;
  private ArrayList<Double> yDeltaArrayList;
  private int run;
  private boolean firstRun;
  private VisionCamera visionCamera;
  private ShortPathToAngle shortPathToAngle;
  private boolean shouldEnd;
  public VisionToGetToTarget() {
    requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldEnd = false;
    xDeltaArrayList = new ArrayList<>();
    xDeltaArrayList.clear();
    yDeltaArrayList = new ArrayList<>();
    yDeltaArrayList.clear();
    visionCamera = new VisionCamera(RobotMap.jevois1);
    camNotifier = new Notifier(new CamRunnable());
    run = 0;
    firstRun = false;
    camNotifier.startPeriodic(0.001);
  
  }
  private class CamRunnable implements Runnable{
    public void run(){
      if(run<10&&!firstRun&&RobotState.isAutonomous()&&!isFinished()){
        double xDelta = visionCamera.getDistance();
        double yDelta = visionCamera.getXOffSet();
        if(xDelta>3&&xDelta<6){
          run++;
          xDeltaArrayList.add(xDelta);
          yDeltaArrayList.add(yDelta);
        }  
      }
      else{
        camNotifier.stop();
      }
     
			
		}
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(run>5&&!firstRun){
      double xSum = 0;
      double ySum = 0;
      double xAverage = 0;
      double yAverage = 0;
      for(int i= 0;i<xDeltaArrayList.size();i++){
        xSum = xSum +xDeltaArrayList.get(i);
      }
      for(int i= 0;i<yDeltaArrayList.size();i++){
        ySum = ySum +yDeltaArrayList.get(i);
      }
      xAverage = xSum/xDeltaArrayList.size();
      yAverage = ySum/yDeltaArrayList.size();
      shortPathToAngle = new ShortPathToAngle(xAverage, yAverage, Pathfinder.d2r(0));
      shortPathToAngle.start();
      System.out.println(xAverage + " " + yAverage);
      firstRun = true;
    }
    else{
      System.out.println(run);
    }
  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(firstRun = true){
      return true;
    }
    return shouldEnd;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    camNotifier.stop();
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
