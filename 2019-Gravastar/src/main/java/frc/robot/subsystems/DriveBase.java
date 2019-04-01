package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import frc.robot.tools.PID;


/**
 *
 */
public class DriveBase extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commnds.
	private PID alignmentPID;
	private double p = 0.015;
	private double i= 0.00015;
	private double d;
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
		
	  
	}
	public void initAlignmentPID(){
		alignmentPID = new PID(p, i, d);
		alignmentPID.setMaxOutput(0.4);
		alignmentPID.setMinOutput(-0.4);
		alignmentPID.setSetPoint(0);

	}
	public void setLowGear(){
	    RobotMap.shifters.set(RobotMap.lowGear);
	}
	public void setHighGear(){
		RobotMap.shifters.set(RobotMap.highGear);
	}
	
	public double getAlignmentPIDOutput(){
		Robot.visionCamera.updateVision();
		if(Timer.getFPGATimestamp()-Robot.visionCamera.lastParseTime>0.25){
			Robot.changeLightColor.changeLedColor(0, 0, 255);
			return 0;
		}
		else{
			Robot.changeLightColor.changeLedColor(0, 255, 0);

			if(Robot.visionCamera.getAngle()>25){
				return 0;
			}
			alignmentPID.updatePID(Robot.visionCamera.getAngle());
			return alignmentPID.getResult();
		}
		
	}
}