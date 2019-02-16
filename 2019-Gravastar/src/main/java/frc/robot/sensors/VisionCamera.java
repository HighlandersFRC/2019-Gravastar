package frc.robot.sensors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.RobotMap;
import edu.wpi.first.hal.util.UncleanStatusException;


public class VisionCamera{
   
   JSONParser parser = new JSONParser();
   SerialPort port;

   public VisionCamera(SerialPort jevois){
      port = jevois;
   }

   public double getAngle(){
     
      double angle = -12.0;

      try{
         
         String unsanatizedString = RobotMap.jevois1.readString();
         System.out.println(unsanatizedString);
           if (unsanatizedString != null){
  
              Object object = parser.parse(unsanatizedString);
  
              JSONObject jsonObject = (JSONObject) object;
         
              if (jsonObject != null){
  
              Long distString = (Long) jsonObject.get("Angle");
  
              angle = Double.valueOf(distString);
  
              }
  
           }
  
        } catch(ParseException e) {
          // e.printStackTrace();
        } catch(UncleanStatusException e) {
          // e.printStackTrace();
        } 
      return angle;
   }

   public double getDistance(){

      double distance = -12.0;
      
      try{
         
       String unsanatizedString = RobotMap.jevois1.readString();
       System.out.println(unsanatizedString);
         if (unsanatizedString != null){

            Object object = parser.parse(unsanatizedString);

            JSONObject jsonObject = (JSONObject) object;
            if (jsonObject != null){
            System.out.println(jsonObject);

            Long distString = (Long) jsonObject.get("Distance");

            distance = Double.valueOf(distString);

            }

         }

      } catch(ParseException e) {
        // e.printStackTrace();
      } catch(UncleanStatusException e) {
        // e.printStackTrace();
      } 

      return distance; 
   } 

   public double getXOffSet(){
     
      double x = -12.0;

      try{
         
         String unsanatizedString = RobotMap.jevois1.readString();
         System.out.println(unsanatizedString);
           if (unsanatizedString != null){
  
              Object object = parser.parse(unsanatizedString);
  
              JSONObject jsonObject = (JSONObject) object;
         
              if (jsonObject != null){
  
              Long distString = (Long) jsonObject.get("PLACEHOLDER");
  
              x = Double.valueOf(distString);
  
              }
  
           }
  
        } catch(ParseException e) {
          // e.printStackTrace();
        } catch(UncleanStatusException e) {
          // e.printStackTrace();
        } 
      return x;
   } 

} 