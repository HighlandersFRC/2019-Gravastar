/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;
import jaci.pathfinder.Pathfinder;

public class ShortPathToAngle extends Command {
  private double xDist;
  private double eAngle;
  private double yDist;
  private double angleError;
  private QuickPathGeneration quickPathGeneration;
  private PurePursuitController purePursuitController;
  private Navx navx;
  private double degreeEndAngle;
  private double startingAngle;
  private boolean firstRun;
  private boolean shouldEnd;
  private double startAngle;
  private boolean shouldSnap;
  private boolean isReversed;
  
  public ShortPathToAngle(double xDisplacement, double yDisplacement, double endAngle, double startingAngle, boolean snapToAngle, boolean reverse) {
    xDist = xDisplacement;
    yDist = yDisplacement;
    eAngle = endAngle;
    startAngle = startingAngle;
    shouldSnap = snapToAngle;
    isReversed = reverse;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.drive.setHighGear();
    quickPathGeneration = new QuickPathGeneration(xDist, yDist, eAngle, isReversed);
    purePursuitController = new PurePursuitController(quickPathGeneration.GeneratePath(),0.8, 4.25, 0.05);
    navx = new Navx(RobotMap.navx);
    startingAngle = navx.currentAngle();
    firstRun = false;
    purePursuitController.start();
    shouldEnd = false;
  }
  public void changePoint(double xDisp, double yDisp, double endAngle){
    xDist = xDisp;
    yDist = yDisp;
    eAngle = endAngle;
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(purePursuitController.isCompleted()){
      if(!firstRun){
        degreeEndAngle = 45*Math.round(startingAngle/45);
        
        System.out.println(degreeEndAngle);
        angleError = degreeEndAngle-navx.currentAngle();
        
        CascadingPIDTurn cascadingPIDTurn= new CascadingPIDTurn(angleError,0.12,0.00070,0.10);
        if(shouldSnap){
          cascadingPIDTurn.start();
        }
        firstRun = true;
      }
    }
    else{

    }
    
  }
  public void forceFinish(){
    shouldEnd = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(navx.currentAngle()-eAngle)<0.5&&purePursuitController.isCompleted()){
      return true;
    }
    else if(!shouldSnap&&purePursuitController.isCompleted()){
      return true;
    }
   
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
