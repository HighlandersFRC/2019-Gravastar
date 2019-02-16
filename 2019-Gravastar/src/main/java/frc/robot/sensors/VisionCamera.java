package frc.robot.sensors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.RobotMap;
import edu.wpi.first.hal.util.UncleanStatusException;


public class VisionCamera{
   
   private SerialPort cam;
   private String sanatizedString = "nothing";
   public VisionCamera(SerialPort port){
      cam = port;
   }
   public String getString(){
      try {

         if(cam.getBytesReceived()>2){
            String unsanatizedString = cam.readString();
            if(unsanatizedString.length()>29||unsanatizedString.isBlank()||unsanatizedString.isEmpty()){
               sanatizedString = unsanatizedString;
            }
         }
      } catch (Exception e) {

         //TODO: handle exception
      }
      
      
      return sanatizedString;
   }
   public double getDistance(){
      try {
         String usedString = this.getString();
        
         return Double.parseDouble(usedString.substring(usedString.indexOf("Distance:")+9, usedString.indexOf("Distance:")+11))/12;
         
         
      } catch (Exception e) {
         return -900.0;
         //TODO: handle exception
      }
   }
   public double getAngle(){
      try {
         String usedString = this.getString();

        
         return Double.parseDouble(usedString.substring(usedString.indexOf("Angle:")+6, usedString.indexOf("Angle:")+11));
         
         
      } catch (Exception e) {
         System.out.println(e.getMessage());
         return -900.0;
         //TODO: handle exception
      }
   }
   


} 