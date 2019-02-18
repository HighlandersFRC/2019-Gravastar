package frc.robot.sensors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import edu.wpi.first.hal.util.UncleanStatusException;




public class VisionCamera {
   
    JSONParser parser = new JSONParser();
    SerialPort port;
    private String sanatizedString = "nothing";
    public double lastParseTime;
   public double distance;
   public double angle;
   private double badAngle = -3000.0;
   private double badDistance = -12.0;
    public VisionCamera(SerialPort jevois) {
       port = jevois;
    }
    public void updateVision(){

      try{
       String jsonString = this.getString();

       if (jsonString != null){
         double tryDistance = parseDistance(jsonString);
         double tryAngle = parseAngle(jsonString);
   
         if (tryDistance != badDistance && tryAngle != badAngle){
            distance = tryDistance;
            angle = tryAngle;

            lastParseTime = Timer.getFPGATimestamp();
         }
      }
   
   }
   catch (Exception e){

   }
}
    public double parseAngle(String jsonString){

 
       try{


   
               Object object = parser.parse(jsonString);
   
               JSONObject jsonObject = (JSONObject) object;
          
               if (jsonObject != null){
   
               double distString = (double) jsonObject.get("Angle");
   
               return Double.valueOf(distString);
   
               
   
            }
   
         } catch(ParseException e) {

         } catch(UncleanStatusException e) {

         } catch(ClassCastException e) {

         }
               return badAngle;
    }
 
    public double parseDistance(String jsonString){

      
             
       try{


 
             Object object = parser.parse(jsonString);
             JSONObject jsonObject = (JSONObject) object;

         
             if (jsonObject != null){ 

             double distString = (double) jsonObject.get("Distance");
             return Double.valueOf(distString);

             }
 
          
 
       } catch(ParseException e) {


       } catch(UncleanStatusException e) {

 
       } catch(ClassCastException e) {

             
         }
 
       return badDistance; 
       
    } 
    
    public double getDistance(){
       return distance;
    }
  
    public double getAngle(){
      return angle;
   }

    public String getString(){
        try {     
          // System.out.println("Raw Read: "+port.readString());
          // System.out.println("Bytes Available: "+port.getBytesReceived());
           if(port.getBytesReceived()>2){
              String unsanatizedString = port.readString();
              //System.out.println("Raw Read: "+unsanatizedString);
              if(unsanatizedString.length()>5&&!unsanatizedString.isBlank()&&!unsanatizedString.isEmpty()){

                 sanatizedString = unsanatizedString;
              }
           }
        } catch (Exception e) {
          // System.out.println("Parse Failed");
        }
    //    System.out.println(sanatizedString);
        //System.out.println("Get String: " + sanatizedString);
        return sanatizedString;   
    } 

    }