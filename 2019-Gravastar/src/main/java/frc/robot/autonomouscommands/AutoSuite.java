package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.autoRoutines.OffHabToLeftSideHatchOne;
import frc.robot.sensors.VisionCamera;
import frc.robot.teleopcommands.ArcadeDrive;
import frc.robot.teleopcommands.DriveTrainStallProtectionController;
import frc.robot.teleopcommands.TankDrive;
import frc.robot.teleopcommands.TeleopArmControl;
import jaci.pathfinder.Pathfinder;

public class AutoSuite {
   // private VisionToGetToTarget visionToGetToTarget= new VisionToGetToTarget();
    // private PurePursuitController purePursuitController ;
    // private QuickPathGeneration quickPathGeneration = new QuickPathGeneration(4, 0, 0);
    // private SendableChooser autoChooser = new SendableChooser<Command>();
    private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
    private Command autoCommand;
    public AutoSuite() {
        arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
		teleopArmControl = new TeleopArmControl();	
    }
    public void startAutoCommandsDriverControl() {
        arcadeDrive.start();
		driveTrainStallProtectionController.start();
		teleopArmControl.start();
    }
    public void startAutoCommandsRobotControl() {
        arcadeDrive.start();
		driveTrainStallProtectionController.start();
		teleopArmControl.start();
    }

    public void endAutoCommands(){
        arcadeDrive.cancel();
		driveTrainStallProtectionController.cancel();
		teleopArmControl.cancel();
    }

}