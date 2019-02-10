/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import frc.robot.Robot;
import frc.robot.teleopcommands.ArcadeDrive;
import frc.robot.teleopcommands.DriveTrainStallProtectionController;
import frc.robot.teleopcommands.TankDrive;
import frc.robot.teleopcommands.TeleopArmControl;

/**
 * Add your docs here.
 */
public class EnableDriverControl {
    private ArcadeDrive arcadeDrive;
	private TankDrive tankDrive;
	private TeleopArmControl teleopArmControl;
	private DriveTrainStallProtectionController driveTrainStallProtectionController;
    public EnableDriverControl(){
  
    }
    public void startDriverControl(){
        Robot.autoSuite.endAutoCommands();
        arcadeDrive = new ArcadeDrive();
        tankDrive = new TankDrive();
        teleopArmControl = new TeleopArmControl();
        driveTrainStallProtectionController = new DriveTrainStallProtectionController();
        tankDrive.start();
        teleopArmControl.start();
        driveTrainStallProtectionController.start();
    }
}
