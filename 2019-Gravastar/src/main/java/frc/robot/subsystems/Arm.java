/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.universalcommands.ArmPositionController;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  private ArmPositionController armPositionController = new ArmPositionController(90);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Arm(){
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intakeBall(){
    RobotMap.intake.set(ControlMode.PercentOutput, -0.75);
  }
  public void shootBall(){
    RobotMap.intake.set(ControlMode.PercentOutput, 1.0);
  }
  public void outTakeBall(){
    RobotMap.intake.set(ControlMode.PercentOutput, 1.0);
  }
  public void intakeWheelsResting(){
    RobotMap.intake.set(ControlMode.PercentOutput, 0);
  }
  public void pushOutHatchMech(){
    RobotMap.hatchPushOutPiston.set(RobotMap.hatchMechIn);
  }
  public void pullInHatchMech(){
    RobotMap.hatchPushOutPiston.set(RobotMap.hatchMechIn);
   
  }
  public void grabHatch(){
    RobotMap.hatchGrabberPiston.set(RobotMap.hatchMechGrab);
  }
  public void releaseHatch(){
    RobotMap.hatchGrabberPiston.set(RobotMap.hatchMechRelease);
  }


}
