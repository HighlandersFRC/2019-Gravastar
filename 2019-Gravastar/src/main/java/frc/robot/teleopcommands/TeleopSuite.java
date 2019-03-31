package frc.robot.teleopcommands;

import frc.robot.universalcommands.ClimbMechanismController;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
	//private ClimbMechanismController climbMechanismController;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
		teleopArmControl = new TeleopArmControl();	
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
