/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Counter;
import frc.robot.RobotConfig;

/**
 * Add your docs here.
 */
public class PWMUltraSonicSensor {
    private Counter counter;
    public PWMUltraSonicSensor(Counter counter){

    }
    public double getDistance(){
        return counter.getPeriod() * RobotConfig.pwmUltraSonicConversionFactor;

    }
}
