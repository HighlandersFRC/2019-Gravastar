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
import jaci.pathfinder.Pathfinder;



/**
 * Add your docs here.
 */
public class VisionCamera {
   SerialPort cam;
   private boolean hasGoodData;
   private String sanatizedString= "Distance:00.000,Angle:0.0000000";
   public VisionCamera(SerialPort port){
      cam = port;
   }
   public String getString(){
      if(cam.getBytesReceived()>10){
         String unsanatizedString = cam.readString();
         if(unsanatizedString.length()<50||unsanatizedString.isBlank()||unsanatizedString.isEmpty()){
            hasGoodData = false;
            return sanatizedString;
            
         }
         else{
            hasGoodData = true;
            sanatizedString = unsanatizedString;
            return sanatizedString;
         }
      }
      else{
         hasGoodData = false;
         return sanatizedString;
      }
     
      
   }
   public double getAngle(){
      String camString = this.getString();
      try{
         if(camString.indexOf('-')>0&&camString.indexOf('-')<13){
            return Double.parseDouble(camString.substring(camString.indexOf(':')+1, 13))/12;
         }
         else{
            return Double.parseDouble(camString.substring(camString.indexOf(':')+1, 13))/12;
          
         }
         
      }
      catch(Exception e){
         System.out.println(camString.substring(camString.lastIndexOf(':')+1, 13));

         return 0;
      }
     
   }
   public double getDistance(){
      String camString = this.getString();
      try{
         return Double.parseDouble(camString.substring(camString.lastIndexOf(':')+1, 53))/12;
      }
      catch(Exception e){
         System.out.println(camString.substring(camString.lastIndexOf(':')+1, 53));
         return 0;
      }
     
   }
   public double getYDisplacement(){
     return Math.cos(Pathfinder.d2r(this.getAngle()))*this.getDistance();
     
   }
   public boolean goodData(){
      return hasGoodData;
   }

}
