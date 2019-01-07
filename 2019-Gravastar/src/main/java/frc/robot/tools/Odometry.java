/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tools;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import jaci.pathfinder.Pathfinder;

public class Odometry extends Command {
  private double theta;
  private double thetaNext;
  private Navx navx;
  private double leftSideNext;
  private double leftSide;
  private double leftDelta;
  private DriveEncoder leftDriveEncoder;
  private double rightSideNext;
  private double rightSide;
  private double rightDelta;
  private DriveEncoder rightDriveEncoder;
  private double centerDelta;
  private double x;
  private double y;
  private double yNext;
  private double xNext;
  private Notifier odometryrunner;
  private double dt;
  private boolean isReversed;
  private boolean finish;
  public Odometry(boolean reversed) {
    leftDriveEncoder = new DriveEncoder(RobotMap.leftDriveLead, RobotMap.leftDriveLead.getSelectedSensorPosition(0));
    rightDriveEncoder = new DriveEncoder(RobotMap.rightDriveLead, RobotMap.rightDriveLead.getSelectedSensorPosition(0));
    navx = new Navx(RobotMap.navx);
    isReversed = reversed;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  public void reverseOdometry(boolean revsered){
    isReversed = revsered;

  }
  private class OdometryRunnable implements Runnable{
    //this is a sperate section of code that runs at a different update rate than the rest, this is necessary to match the dt
     public void run(){
      if(isReversed){
        leftSideNext = leftDriveEncoder.getDistance();
        rightSideNext = rightDriveEncoder.getDistance();
        thetaNext = navx.currentReverseYaw();
        leftDelta = -(leftSideNext-leftSide);
        rightDelta = -(rightSideNext-rightSide);
        centerDelta = (leftDelta+rightDelta)/2;
        xNext = x-centerDelta*Math.cos(Pathfinder.d2r(thetaNext));
        yNext = y-centerDelta*Math.sin(Pathfinder.d2r(thetaNext));
        x = xNext;
        y = yNext;
        theta = thetaNext;
        leftSide = leftSideNext;
        rightSide = rightSideNext;
      }
      else{
        leftSideNext = leftDriveEncoder.getDistance();
        rightSideNext = rightDriveEncoder.getDistance();
        thetaNext = navx.currentYaw();
        leftDelta = (leftSideNext-leftSide);
        rightDelta = (rightSideNext-rightSide);
        centerDelta = (leftDelta+rightDelta)/2;
        xNext = x+centerDelta*Math.cos(Pathfinder.d2r(thetaNext));
        yNext = y+centerDelta*Math.sin(Pathfinder.d2r(thetaNext));
        x = xNext;
        y = yNext;
        theta = thetaNext;
        leftSide = leftSideNext;
        rightSide = rightSideNext;
      }
     }
 
   }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    navx.softResetAngle(navx.currentAngle());
    navx.softResetYaw(navx.currentYaw());
    leftDriveEncoder.softReset();
    rightDriveEncoder.softReset();
    odometryrunner = new Notifier(new OdometryRunnable());
    dt = 0.005;
    odometryrunner.startPeriodic(dt);
    finish = false;
  }
  public void endOdmetry(){
    finish = true;
  }

  public void zero(){
    x =0;
    y = 0;
    theta = 0;
    navx.softResetYaw(navx.currentYaw());
    leftDriveEncoder.softReset();
    rightDriveEncoder.softReset();
    leftSide = 0;
    leftSideNext = 0;
    rightSide = 0;
    rightSideNext= 0;
  }
  public double getX(){
    return x;
  }
  public double getY(){
    return y;
  }
  public double gettheta(){
    return theta;
  }
  public void setX(double xValue){
    x = xValue;
  }
  public void setY(double yValue){
    y = yValue;
  }
  public void setTheta(double thetaValue){
    theta = thetaValue;
  }
  public void setReversed(boolean reversed){
    isReversed = reversed;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(finish){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    odometryrunner.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  
  }
}
