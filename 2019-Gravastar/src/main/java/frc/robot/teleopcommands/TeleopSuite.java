package frc.robot.teleopcommands;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		tankDrive = new TankDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();	
	}
	public void startTeleopCommands() {
		arcadeDrive.start();
		//tankDrive.start();
		driveTrainStallProtectionController.start();
	}
	public void endTeleopCommands(){
		arcadeDrive.cancel();
		//tankDrive.cancel();
		driveTrainStallProtectionController.cancel();
	}

}
