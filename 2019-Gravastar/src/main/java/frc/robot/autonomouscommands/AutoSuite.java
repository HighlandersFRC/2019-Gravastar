package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.ArcadeDrive;
import frc.robot.teleopcommands.DriveTrainStallProtectionController;
import frc.robot.teleopcommands.TankDrive;
import frc.robot.teleopcommands.TeleopArmControl;
import frc.robot.universalcommands.DriveTrainController;
import jaci.pathfinder.Pathfinder;

public class AutoSuite {
   
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
    private Command autoCommand;
    private CascadingPIDUltrasonicAlignment cascadingPIDUltrasonicAlignment;
    private UltrasoundAlineandGetTo ultrasoundAlineandGetTo;
    private ShortPathToAngle shortPathToAngle = new ShortPathToAngle(5, -2, 0, 0, false, false);
    private VisionToGetToTarget visionToGetToTarget;
    private DriveTrainController driveTrainController;
    public AutoSuite() {
    
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
        teleopArmControl = new TeleopArmControl();	
        visionToGetToTarget = new VisionToGetToTarget(false);
        driveTrainController = new DriveTrainController();
    }
    public void startAutoCommandsDriverControl() {
		driveTrainStallProtectionController.start();
        teleopArmControl.start();
        driveTrainController.start();
    }
    public void startAutoCommandsRobotControl() {
      
    }

    public void endAutoCommands(){
		driveTrainStallProtectionController.cancel();
		teleopArmControl.cancel();
    }

}