package frc.robot.autonomouscommands;

import frc.robot.RobotMap;
import frc.robot.autoRoutines.OffHabToLeftSideHatchOne;
import jaci.pathfinder.Pathfinder;

public class AutoSuite {
    private OffHabToLeftSideHatchOne habToLeftSideHatchOne;
    private ShortPathToAngle shortPathToAngle;
    public AutoSuite() {
        
        shortPathToAngle = new ShortPathToAngle(5.0,Pathfinder.d2r(0),Pathfinder.d2r(0));
    }
    public void startAutoCommands() {
        shortPathToAngle.start();
    }
    public void endAutoCommands(){
        shortPathToAngle.cancel();
       
    }

}