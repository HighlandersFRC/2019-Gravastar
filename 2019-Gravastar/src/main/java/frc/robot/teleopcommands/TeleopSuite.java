package frc.robot.teleopcommands;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		driveTrainStallProtectionController = new DriveTrainStallProtectionController();	
	}
	public void startTeleopCommands() {
		arcadeDrive.start();
		driveTrainStallProtectionController.start();
	}
	public void endTeleopCommands(){
		arcadeDrive.cancel();
		driveTrainStallProtectionController.cancel();
	}

}
