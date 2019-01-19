package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 *
 */
public class DriveBase extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
		
	  
	}
	public void setLowGear(){
	   // RobotMap.shifters.set(RobotMap.lowGear);
	}
	public void setHighGear(){
	   // RobotMap.shifters.set(RobotMap.highGear);
	}
}