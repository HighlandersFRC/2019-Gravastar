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
    RobotMap.intake.set(ControlMode.PercentOutput, -0.6);
  }
  public void shootBall(){
    RobotMap.intake.set(ControlMode.PercentOutput, 0.8);
  }
  public void intakeWheelsResting(){
    RobotMap.intake.set(ControlMode.PercentOutput, 0);
  }
  public void pushOutHatch(){
    RobotMap.hatchPiston1.set(RobotMap.pushOut);
    RobotMap.hatchPiston2.set(RobotMap.pushOut);
    RobotMap.hatchPiston3.set(RobotMap.pushOut);
  }
  public void pullInAllHatchPistons(){
    RobotMap.hatchPiston1.set(RobotMap.in);
    RobotMap.hatchPiston2.set(RobotMap.in);
    RobotMap.hatchPiston3.set(RobotMap.in);
  }

}
