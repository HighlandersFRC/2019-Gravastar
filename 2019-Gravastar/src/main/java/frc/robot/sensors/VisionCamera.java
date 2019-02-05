/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;



import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.SerialPort;



/**
 * Add your docs here.
 */
public class VisionCamera {
   SerialPort cam;
   public VisionCamera(SerialPort port){
      cam = port;
   }
   public String getDistance(){
      String unsanatizedString = cam.readString();
      if(unsanatizedString.length()<8||unsanatizedString.isBlank()||unsanatizedString.isEmpty()||unsanatizedString.indexOf(':')<1||unsanatizedString.indexOf(',')<1||unsanatizedString.indexOf(',')<unsanatizedString.indexOf(':')){
         return "-12.0";
      }
      else{
         return unsanatizedString.substring(unsanatizedString.indexOf(':'), unsanatizedString.indexOf(','));
      }
      
   }

}
