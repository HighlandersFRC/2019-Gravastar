package frc.robot.teleopcommands;

import frc.robot.universalcommands.ClimbMechanismController;
import frc.robot.universalcommands.DriveTrainController;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private TeleopArmControl teleopArmControl;
	private DriveTrainController driveTrainController;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
	//private ClimbMechanismController climbMechanismController;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
		teleopArmControl = new TeleopArmControl();	
		driveTrainController = new DriveTrainController();
	}
	public void startTeleopCommands() {
		driveTrainStallProtectionController.start();
		teleopArmControl.start();
		arcadeDrive.start();
		//tankDrive.start();
	}
	public void endTeleopCommands(){
		driveTrainStallProtectionController.cancel();
		teleopArmControl.cancel();
		arcadeDrive.cancel();
		//tankDrive.cancel();
	}

}
