/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.UltrasonicSensor;

public class UltrasoundGetToTarget extends Command {
  private UltrasonicSensor sensor1;
  private UltrasonicSensor sensor2;
  private double distanceAverage;
  private double distanceSum;
  private int runCounter;
  private Boolean firstRun;
  private ShortPathToAngle shortPathToAngle;
  private ArrayList distanceArrayList = new ArrayList<Double>();
  public UltrasoundGetToTarget(UltrasonicSensor ultraSound1, UltrasonicSensor ultraSound2) {
    sensor1 = ultraSound1;
    sensor2 = ultraSound2;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    distanceArrayList.clear();
    firstRun = false;
    runCounter = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    distanceSum = distanceSum +(sensor1.getDistance() + sensor2.getDistance())/2;
    runCounter++;
    if(runCounter>=10&&!firstRun){
      distanceAverage = distanceSum/10;
      System.out.println(distanceAverage);
      shortPathToAngle = new ShortPathToAngle(distanceAverage, 0, 0);
      shortPathToAngle.start();
      firstRun = true;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(sensor1.getDistance()<0||sensor2.getDistance()<0){
      return true;
    }
    return firstRun;
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
