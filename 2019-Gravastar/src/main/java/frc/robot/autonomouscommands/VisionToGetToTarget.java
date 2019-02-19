/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*package frc.robot.autonomouscommands;

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
  private int succesfulRunCounter;
  private int runCounter;


  private boolean firstRun = false;
  private VisionCamera visionCamera;
  private ShortPathToAngle shortPathToAngle;
  private boolean shouldEnd;
  private double previousAngle;
  private double previousDistance;
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
    succesfulRunCounter = 0;
    firstRun = false;
    camNotifier.startPeriodic(0.05);
  
  }
  private class CamRunnable implements Runnable{
  
    public void run(){
      double distance = visionCamera.getDistance();
      double angle = Pathfinder.d2r(visionCamera.getAngle());
      if(succesfulRunCounter==0){
        previousAngle = angle;
        previousDistance = distance;
      }
      if(Math.abs(angle)< 0.2879793 && distance>1.5){
        if(Math.abs(previousDistance-distance)<0.3&&Math.abs(previousAngle-angle)<1){
          if(succesfulRunCounter<10&&!firstRun&&!isFinished()){
            double xDelta = Math.cos(angle)*distance;
            double yDelta = Math.sin(2*angle)*distance;
            if(xDelta>1&&xDelta<6){
              succesfulRunCounter++;
              xDeltaArrayList.add(xDelta);
              yDeltaArrayList.add(yDelta);
              previousAngle = angle;
              previousAngle = distance;
            }  
          }
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
    if(succesfulRunCounter>5&&!firstRun){
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
      System.out.println(xAverage + " " + yAverage);
      shortPathToAngle = new ShortPathToAngle(xAverage-0.25, yAverage+0.75, Pathfinder.d2r(0));
      shortPathToAngle.start();
      firstRun = true;
    }
    else{
    }
    runCounter++;

  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(firstRun == true){
      return true;
    }
    if(runCounter>30){
      return true;
    }
    return false;
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
}*/
