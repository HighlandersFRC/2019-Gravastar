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
    private VisionToGetToTarget visionToGetToTarget;
    //private OffHabToLeftSideHatchOne offHabToLeftSideHatchOne;
    private SendableChooser autoChooser = new SendableChooser<Command>();
    private Command autoCommand;
    public AutoSuite() {
    
        visionToGetToTarget = new VisionToGetToTarget();
       // offHabToLeftSideHatchOne = new OffHabToLeftSideHatchOne();
        //autoChooser.addOption("OffHabToLeftSideHatchOne", offHabToLeftSideHatchOne);
        autoChooser.addOption("VisionToGetToTarget", visionToGetToTarget);
    }
    public void startAutoCommands() {
       // autoCommand = (Command) autoChooser.getSelected();
        //autoCommand.start();
       

    }
    public void endAutoCommands(){
        //autoCommand.cancel();
    }

}