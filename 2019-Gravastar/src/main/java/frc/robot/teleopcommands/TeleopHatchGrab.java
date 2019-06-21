/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.universalcommands.PullInHatchMech;
import frc.robot.universalcommands.PushOutHatchMech;
import frc.robot.universalcommands.ReleaseHatchGrabbers;
import frc.robot.universalcommands.TenseHatchGrabbers;
public class TeleopHatchGrab extends Command {


  public TeleopHatchGrab() {
    

    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.arm.pushOutHatchMech();
    RobotMap.arm.releaseHatchGrabbers();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(OI.pilotController.getBumper(Hand.kLeft));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !OI.pilotController.getBumper(Hand.kLeft);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.arm.pullInHatchMech();
    RobotMap.arm.tenseHatchGrabbers();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
