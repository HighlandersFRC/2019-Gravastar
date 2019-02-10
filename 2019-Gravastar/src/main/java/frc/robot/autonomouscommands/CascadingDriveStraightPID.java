/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;
import frc.robot.tools.DriveTrainVelocityPID;
import frc.robot.tools.PID;
public class CascadingDriveStraightPID extends Command {
  private DriveTrainVelocityPID leftVelocityPID = new DriveTrainVelocityPID(0, RobotMap.leftDriveLead, 2, 0.0402026, 0.18, 0.0006, 0.80);
  private DriveTrainVelocityPID rightVelocityPID = new DriveTrainVelocityPID(0, RobotMap.rightDriveLead, 2,  0.0406258, 0.18, 0.0006, 0.80);
  private PID stayStraightPID = new PID(0.080, 0.00001, 0.001);
  private Navx straightNavx;
  private double desiredSpeed;
  private double startTime;
  private double deltaTime;
  private boolean shouldEnd;
  public CascadingDriveStraightPID(double speed, double time) {
    desiredSpeed = speed;
    deltaTime = time;
    straightNavx = new Navx(RobotMap.navx);
    requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldEnd = false;
    stayStraightPID.setMaxOutput(5);
    stayStraightPID.setMinOutput(-5);
    stayStraightPID.setSetPoint(0);
    leftVelocityPID.start();
    rightVelocityPID.start();
    straightNavx.softResetYaw();
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("yaw", straightNavx.currentYaw());
    SmartDashboard.putNumber("i",1);
    stayStraightPID.updatePID(straightNavx.currentYaw());
    leftVelocityPID.changeDesiredSpeed(desiredSpeed - stayStraightPID.getResult());
    rightVelocityPID.changeDesiredSpeed(desiredSpeed + stayStraightPID.getResult());
  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Timer.getFPGATimestamp()-startTime>deltaTime){
      return true;
    }
    return shouldEnd;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftVelocityPID.endPID();
    rightVelocityPID.endPID();
    Robot.stopMotors.stopDriveTrainMotors();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
