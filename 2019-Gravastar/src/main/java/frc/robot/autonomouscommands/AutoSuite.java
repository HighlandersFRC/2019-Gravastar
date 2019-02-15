package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.autoRoutines.OffHabToLeftSideHatchOne;
import frc.robot.sensors.VisionCamera;
import jaci.pathfinder.Pathfinder;

public class AutoSuite {
    private VisionToGetToTarget visionToGetToTarget= new VisionToGetToTarget();
    private PurePursuitController purePursuitController ;
    private QuickPathGeneration quickPathGeneration = new QuickPathGeneration(4, 0, 0);
    private SendableChooser autoChooser = new SendableChooser<Command>();
    private Command autoCommand;
    public AutoSuite() {
    
    }
    public void startAutoCommands() {
       visionToGetToTarget.start();
       

    }
    public void endAutoCommands(){
        //autoCommand.cancel();
    }

}