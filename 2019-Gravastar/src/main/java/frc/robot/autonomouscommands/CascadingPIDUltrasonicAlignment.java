/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import frc.robot.sensors.UltrasonicSensor;
import frc.robot.tools.DriveTrainVelocityPID;
import frc.robot.tools.PID;

public class CascadingPIDUltrasonicAlignment extends Command {
  private Navx navx;
  private DriveTrainVelocityPID leftDriveTrainVelocityPID;
  private DriveTrainVelocityPID rightDriveTrainVelocityPID;
  private PID turnPID;
  private double desiredAngle;
  private DriveEncoder leftSideDriveEncoder;
  private DriveEncoder rightSideDriveEncoder;
  private double p;
  private double i;
  private double d;
  private UltrasonicSensor ultraSonic1;
  private UltrasonicSensor ultraSonic2;
  private boolean shouldEnd;
  public CascadingPIDUltrasonicAlignment(double kp, double ki, double kd, UltrasonicSensor sonic1, UltrasonicSensor sonic2) {
    p = kp;
    i = ki;
    d = kd;
    ultraSonic1 = sonic1;
    ultraSonic2 = sonic2;
    requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldEnd = false;
    leftDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.leftDriveLead, 0,0.0402026, 0.18, 0.0004, 0.8);
    rightDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.rightDriveLead, 0,0.0406258, 0.18, 0.0004, 0.8);
    turnPID =  new PID(p,i,d);
    leftSideDriveEncoder = new DriveEncoder(RobotMap.leftDriveLead, RobotMap.leftDriveLead.getSelectedSensorPosition(0));
    rightSideDriveEncoder = new DriveEncoder(RobotMap.rightDriveLead, RobotMap.rightDriveLead.getSelectedSensorPosition(0));
    turnPID.setMaxOutput(RobotConfig.robotMaxVelocity);
    turnPID.setMinOutput(-RobotConfig.robotMaxVelocity);
    turnPID.setSetPoint(0);
    leftDriveTrainVelocityPID.start();
    rightDriveTrainVelocityPID.start();
  }
 

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    turnPID.updatePID(ultraSonic1.getDistance()-ultraSonic2.getDistance());
    leftDriveTrainVelocityPID.changeDesiredSpeed(turnPID.getResult());
    rightDriveTrainVelocityPID.changeDesiredSpeed(-turnPID.getResult());
  }
  public void forceFinish(){
    shouldEnd = true;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(ultraSonic1.getDistance()-ultraSonic2.getDistance()<0.01){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftDriveTrainVelocityPID.cancel();
    rightDriveTrainVelocityPID.cancel();
    Robot.stopMotors.stopDriveTrainMotors();

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
