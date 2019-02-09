/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;
import jaci.pathfinder.Pathfinder;

public class ShortPathToAngle extends Command {
  private double dist;
  private double rAngle;
  private double eAngle;
  private double angleError;
  private QuickPathGeneration quickPathGeneration;
  private PurePursuitController purePursuitController;
  private Navx navx;
  private double degreeEndAngle;
  private double startingAngle;
  
  private boolean firstRun;
  
  
  public ShortPathToAngle(double distance, double robotAngle, double endAngle) {
    dist = distance;
    rAngle = robotAngle;
    eAngle = endAngle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    quickPathGeneration = new QuickPathGeneration(dist, rAngle, eAngle);
    purePursuitController = new PurePursuitController(quickPathGeneration.GeneratePath(), 1.0, 4, 0.05);
    navx = new Navx(RobotMap.navx);
    startingAngle = navx.currentAngle();
    firstRun = false;
    degreeEndAngle = Pathfinder.r2d(eAngle);
    purePursuitController.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(purePursuitController.isCompleted()){
      if(!firstRun){
        angleError = degreeEndAngle-navx.currentAngle();
      
        CascadingPIDTurn cascadingPIDTurn= new CascadingPIDTurn(angleError,0.08,0.00085,0.00006);;
        cascadingPIDTurn.start();
        firstRun = true;
      }
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(navx.currentAngle()-eAngle)<5&&purePursuitController.isCompleted()){
      return true;
    }
    else{
      return false;
    }
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
