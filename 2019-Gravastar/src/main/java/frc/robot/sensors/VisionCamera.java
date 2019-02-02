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
    private String sanitizedData1;
    private String sanitizedData2;
    private Double rawDistance;
    private double actualDistance;
    private double x;
    private double y;
    public VisionCamera(SerialPort port){
        camera = port;

    }
    public String getDistance(){
        rawData = camera.readString();
        if(!rawData.isBlank()){
            sanitizedData1 = rawData;
        }
        if(sanitizedData1.length()>6&&sanitizedData1.indexOf('.')>=2){
            sanitizedData2 = sanitizedData1.substring(0,sanitizedData1.indexOf('.')+4);
            
        }
        /*if(rawDistance>0){
            actualDistance = rawDistance;
        }*/
        return sanitizedData2;
    }
   
}
