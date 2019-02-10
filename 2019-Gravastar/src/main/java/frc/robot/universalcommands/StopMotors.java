/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class StopMotors {
    public StopMotors(){

    }
    public void stopDriveTrainMotors(){
        for(TalonSRX talon : RobotMap.driveMotors){
            talon.set(ControlMode.PercentOutput, 0);
        }
    }
    public void stopArmMotors(){
        for(TalonSRX talon : RobotMap.armMotors){
            talon.set(ControlMode.PercentOutput, 0);
        }
    }
    public void stopAllMotors(){
        for(TalonSRX talon : RobotMap.allMotors){
            talon.set(ControlMode.PercentOutput, 0);
        }
    }
}
