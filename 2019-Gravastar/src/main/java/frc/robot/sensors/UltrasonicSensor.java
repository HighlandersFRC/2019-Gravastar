/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotConfig;

/**
 * Add your docs here.
 */
public class UltrasonicSensor {
    AnalogInput ultraSonic;
    public UltrasonicSensor(AnalogInput input){
        ultraSonic = input;
    }
    public double getDistance(){
        if(ultraSonic.getAverageValue()*RobotConfig.ultraSonicConversionFactor>1.2){
            return ultraSonic.getValue()*RobotConfig.ultraSonicConversionFactor;
        }  
        else{
            return -1;
        }
        
    }
    public double getVoltage(){
        return ultraSonic.getVoltage();
    }
}
