package frc.robot.sensors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.RobotMap;
import edu.wpi.first.hal.util.UncleanStatusException;




public class VisionCamera {
   
    JSONParser parser = new JSONParser();
    SerialPort port;
    private String sanatizedString = "nothing";

    public VisionCamera(SerialPort jevois) {
       port = jevois;
    }
    
    public double getAngle(){
      
       double angle = -12.0;
 
       try{
          
          String jsonString = this.getString();
            if (jsonString != null){
   
               Object object = parser.parse(jsonString);
   
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
         } catch(ClassCastException e) {

         }
               return angle;
    }
 
    public double getDistance(){
 
        double distance = -12.0;
       
       try{
          
        String jsonString = this.getString();
          if (jsonString != null){
 
             Object object = parser.parse(jsonString);

             JSONObject jsonObject = (JSONObject) object;
             if (jsonObject != null){ 
             Long distString = (Long) jsonObject.get("Distance");
 
             distance = Double.valueOf(distString);

 
             }
 
          }
 
       } catch(ParseException e) {
         // e.printStackTrace();
       } catch(UncleanStatusException e) {
         // e.printStackTrace();
       } catch(ClassCastException e) {
             
         }
 
       return distance; 
    } 
  
    public String getString(){
        try {     
           if(port.getBytesReceived()>2){
              String unsanatizedString = port.readString();
              if(unsanatizedString.length()>18||unsanatizedString.isBlank()||unsanatizedString.isEmpty()){

                 sanatizedString = unsanatizedString;
              }
           }
        } catch (Exception e) {

        }
        return sanatizedString;   
    }
    }