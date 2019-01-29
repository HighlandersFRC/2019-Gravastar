/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ActuateAllHatchPistons {

    public ActuateAllHatchPistons(){

    }
    public void actuatePistons(DoubleSolenoid.Value position){
        RobotMap.hatchPiston1.set(position);
        RobotMap.hatchPiston2.set(position);
        RobotMap.hatchPiston3.set(position);
    }
}
