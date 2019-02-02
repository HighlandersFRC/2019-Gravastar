package frc.robot.teleopcommands;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();
		teleopArmControl = new TeleopArmControl();	
	}
	public void startTeleopCommands() {
		arcadeDrive.start();
		//tankDrive.start();
		driveTrainStallProtectionController.start();
		teleopArmControl.start();
	}
	public void endTeleopCommands(){
		arcadeDrive.cancel();
		//tankDrive.cancel();
		driveTrainStallProtectionController.cancel();
		teleopArmControl.cancel();
	}

}
