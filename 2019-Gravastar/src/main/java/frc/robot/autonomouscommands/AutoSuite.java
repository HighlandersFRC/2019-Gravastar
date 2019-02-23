package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

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
    private CascadingPIDUltrasonicAlignment cascadingPIDUltrasonicAlignment;
    private UltrasoundAlineandGetTo ultrasoundAlineandGetTo;
    //private ShortPathToAngle shortPathToAngle = new ShortPathToAngle(5, 3, 0);
    private VisionToGetToTarget visionToGetToTarget;
    private PurePursuitController purePursuitController = new PurePursuitController(RobotMap.universalPathList.pathSetup, 0.9, 2.25, 0.05);
    public AutoSuite() {
        arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
        teleopArmControl = new TeleopArmControl();	
        visionToGetToTarget = new VisionToGetToTarget();
    }
    public void startAutoCommandsDriverControl() {
        //arcadeDrive.start();
        tankDrive.start();
		driveTrainStallProtectionController.start();
		teleopArmControl.start();
    }
    public void startAutoCommandsRobotControl() {
       visionToGetToTarget.start();
      //purePursuitController.start();
      //shortPathToAngle.start();
    }

    public void endAutoCommands(){
        arcadeDrive.cancel();
		driveTrainStallProtectionController.cancel();
		teleopArmControl.cancel();
    }

}