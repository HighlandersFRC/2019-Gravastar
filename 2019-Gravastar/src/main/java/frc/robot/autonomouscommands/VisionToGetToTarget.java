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
import jaci.pathfinder.Pathfinder;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;

public class VisionToGetToTarget extends Command {
  private Notifier camNotifier;
  private ArrayList<Double> distanceArrayList;
  private ArrayList<Double> angleArrayList;
  private int run;
  private boolean firstRun;
  private VisionCamera visionCamera;
  private ShortPathToAngle shortPathToAngle;
  public VisionToGetToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    distanceArrayList = new ArrayList<>();
    distanceArrayList.clear();
    angleArrayList = new ArrayList<>();
    angleArrayList.clear();
    visionCamera = new VisionCamera(RobotMap.jevois1);
    camNotifier = new Notifier(new CamRunnable());
    run = 0;
    firstRun = false;
    camNotifier.startPeriodic(0.001);
  
  }
  private class CamRunnable implements Runnable{
    public void run(){
      if(run<10&&!firstRun&&RobotState.isAutonomous()&&!isFinished()){
        double distance = visionCamera.getDistance();
        //double angle = visionCamera.getAngle();
        if(distance>3&&distance<6){
          run++;
         // angleArrayList.add(angle);
          distanceArrayList.add(distance);
        }
        
        //System.out.println(run +"angle " + angle + "distance " + distance);
  
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
      double distanceSum = 0;
      double angleSum = 0;
      double distanceAverage = 0;
      double angleAverage = 0;
      for(int i= 0;i<distanceArrayList.size();i++){
        distanceSum = distanceSum +distanceArrayList.get(i);
      }
      for(int i= 0;i<angleArrayList.size();i++){
        angleSum = distanceSum +angleArrayList.get(i);
      }
      distanceAverage = distanceSum/distanceArrayList.size();
      angleAverage = angleSum/angleArrayList.size();
      shortPathToAngle = new ShortPathToAngle(distanceAverage, Pathfinder.d2r(angleAverage), Pathfinder.d2r(0));
      shortPathToAngle.start();
      System.out.println(distanceAverage + " " + angleAverage);
      firstRun = true;
    }
    else{
      System.out.println(run);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return firstRun;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    camNotifier.stop();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
