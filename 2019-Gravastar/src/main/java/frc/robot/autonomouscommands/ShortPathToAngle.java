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
  
  public ShortPathToAngle(double xDisplacement, double yDisplacement, double endAngle) {
    xDist = xDisplacement;
    yDist = yDisplacement;
    eAngle = endAngle;
    requires(RobotMap.drive);

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    quickPathGeneration = new QuickPathGeneration(xDist, yDist, eAngle);
    purePursuitController = new PurePursuitController(quickPathGeneration.GeneratePath(), 1.0, 4, 0.05);
    navx = new Navx(RobotMap.navx);
    startingAngle = navx.currentAngle();
    firstRun = false;
    degreeEndAngle = Pathfinder.r2d(eAngle);
    purePursuitController.start();
    shouldEnd = false;
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
  public void forceFinish(){
    shouldEnd = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(navx.currentAngle()-eAngle)<5&&purePursuitController.isCompleted()){
      return true;
    }
   
    return shouldEnd;

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
