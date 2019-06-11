package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.ArcadeDrive;
import frc.robot.teleopcommands.DriveTrainStallProtectionController;
import frc.robot.teleopcommands.TankDrive;
import frc.robot.teleopcommands.TeleopArmControl;
import jaci.pathfinder.Pathfinder;
import frc.robot.OI;

public class AutoSuite {
   
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
    private Command autoCommand;
    private CascadingPIDUltrasonicAlignment cascadingPIDUltrasonicAlignment;
    private UltrasoundAlineandGetTo ultrasoundAlineandGetTo;
    private ArcadeDrive arcadeDrive;
    private PurePursuitController purePursuitController = new PurePursuitController(PathList.test1Path, 2.7, 4.25, 0.05);
    public AutoSuite() {
    
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
        teleopArmControl = new TeleopArmControl();	
        arcadeDrive = new ArcadeDrive();
    }
    public void startAutoCommandsDriverControl() {

		driveTrainStallProtectionController.start();
        teleopArmControl.start();
        arcadeDrive.start();
        
    }
    public void startAutoCommandsRobotControl() {
        purePursuitController.start();
      
    }

    public void endAutoCommands(){
		driveTrainStallProtectionController.cancel();
        teleopArmControl.cancel();
        arcadeDrive.cancel();
    }

}