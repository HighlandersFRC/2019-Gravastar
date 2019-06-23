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
import frc.robot.autonomouscommands.GrabHatchAndReturnAuto;;

public class AutoSuite {
   
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
    private Command autoCommand;
    private ArcadeDrive arcadeDrive;
    private PurePursuitController purePursuitController = new PurePursuitController(PathList.test1Path, 0.8, 5.0, 0.05);
    private VisionAutoHatchPickup visionAutoHatchPickup = new VisionAutoHatchPickup();
    private GrabHatchAndReturnAuto testAuto = new GrabHatchAndReturnAuto();
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
        testAuto.start();
    }

    public void endAutoCommands(){
		driveTrainStallProtectionController.cancel();
        teleopArmControl.cancel();
        arcadeDrive.cancel();
    }

}