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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class VisionToGetToTarget extends Command {
  private Notifier camNotifier;
  private ArrayList<Double> xDeltaArrayList;
  private ArrayList<Double> yDeltaArrayList;
  private int succesfulRunCounter;
  private int runCounter;


  private boolean firstRun = false;
  public ShortPathToAngle shortPathToAngle;
  private boolean shouldEnd;
  private double previousAngle;
  private double previousDistance;
  private double startTime;
  private boolean isReversed;
  private DoubleSolenoid.Value value;
  public VisionToGetToTarget(boolean reverse) {
    requires(RobotMap.drive);
    isReversed = reverse;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    shouldEnd = false;
    xDeltaArrayList = new ArrayList<>();
    xDeltaArrayList.clear();
    yDeltaArrayList = new ArrayList<>();
    yDeltaArrayList.clear();
 
	
    succesfulRunCounter = 0;
    runCounter = 0;
    firstRun = false;
   
    Robot.visionCamera.updateVision();

  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double distance = Robot.visionCamera.getDistance();
    double angle = Math.toRadians(Robot.visionCamera.getAngle());
    double totalUltraSonicDistance = (RobotMap.mainUltrasonicSensor3.getDistance() + RobotMap.mainUltrasonicSensor4.getDistance())/2;
    if(RobotMap.mainUltrasonicSensor4.getDistance()<8&&RobotMap.mainUltrasonicSensor4.getDistance()>2&&RobotMap.mainUltrasonicSensor3.getDistance()<8&&RobotMap.mainUltrasonicSensor3.getDistance()>0){
      distance = (distance + totalUltraSonicDistance)/2;
    }
    distance = distance;  
    if(succesfulRunCounter<20&&!firstRun&&!isFinished()){
      double xDelta = distance;
      double yDelta = Math.sin(angle)*distance;
      if(xDelta>0.1&&xDelta<20&&Math.abs(angle)<0.78539){
        succesfulRunCounter++;
        xDeltaArrayList.add(xDelta);
        yDeltaArrayList.add(yDelta);
        previousAngle = angle;
        previousAngle = distance;
      }  
    }
     
    
   
    if(succesfulRunCounter>10&&!firstRun){
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
      shortPathToAngle = new ShortPathToAngle(xAverage+0.7, yAverage, angle,RobotMap.mainNavx.currentAngle(), true, isReversed);
      
     
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
    return false;
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
    this.end();
  }
}
