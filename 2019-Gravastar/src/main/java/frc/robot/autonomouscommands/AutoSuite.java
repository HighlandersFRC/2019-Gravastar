package frc.robot.autonomouscommands;

import frc.robot.RobotMap;
import frc.robot.autoRoutines.OffHabToLeftSideHatchOne;
import frc.robot.sensors.VisionCamera;
import jaci.pathfinder.Pathfinder;

public class AutoSuite {
    private VisionToGetToTarget visionToGetToTarget;

    public AutoSuite() {
        visionToGetToTarget = new VisionToGetToTarget();
        
        
    }
    public void startAutoCommands() {
        visionToGetToTarget.start();
       

    }
    public void endAutoCommands(){
       
    }

}