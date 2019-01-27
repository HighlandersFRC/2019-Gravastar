/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * Add your docs here.
 */
public class VisionCamera {
    private SerialPort camera;
    private String rawData;
    private String sanitizedData;
    private Double distance;
    public VisionCamera(SerialPort port){
        camera = port;

    }
    public double getDist(){
        rawData = camera.readString();
        if(!rawData.isEmpty()){
            sanitizedData = rawData;
            distance = Double.parseDouble(sanitizedData);
        }
        else{
            distance = -1.0;
        }
        return distance;
    }
}
