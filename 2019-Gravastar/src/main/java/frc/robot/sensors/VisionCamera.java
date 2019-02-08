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
   private String sanatizedString= "Distance:00.000,Angle:0.0000000";
   public VisionCamera(SerialPort port){
      cam = port;
   }
   public String getString(){
      if(cam.getBytesReceived()>10){
         String unsanatizedString = cam.readString();
         if(unsanatizedString.length()<30||unsanatizedString.isBlank()||unsanatizedString.isEmpty()){
            return sanatizedString;
         }
         else{
            sanatizedString = unsanatizedString;
            return sanatizedString;
         }
      }
      else{
         return sanatizedString;
      }
     
      
   }
   public double getAngle(){
      try{
         return Double.parseDouble(this.getString().substring(23, 27));
      }
      catch(Exception e){
         return -12;
      }
     
   }
   public double getDistance(){
      try{
         return Double.parseDouble(this.getString().substring(9, 14))/12;
      }
      catch(Exception e){
         return -12;
      }
     
   }

}
