package frc.robot.universalcommands;

import frc.robot.RobotMap;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ChangeLightColor{
	private double red;
	private double green;
	private double blue;

    public ChangeLightColor(double R, double G, double B) {
    	red=R;
    	green = G;
    	blue = B;
    	RobotMap.canifier.setLEDOutput(green,CANifier.LEDChannel.LEDChannelA);
		RobotMap.canifier.setLEDOutput(blue,CANifier.LEDChannel.LEDChannelB);
		RobotMap.canifier.setLEDOutput(red,CANifier.LEDChannel.LEDChannelC);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
    public void changeLedColor(double R,double G,double B) {
    	red=R;
    	green = G;
    	blue = B;
    	RobotMap.canifier.setLEDOutput(green,CANifier.LEDChannel.LEDChannelA);
		RobotMap.canifier.setLEDOutput(blue,CANifier.LEDChannel.LEDChannelB);
		RobotMap.canifier.setLEDOutput(red,CANifier.LEDChannel.LEDChannelC);
    	
    }
}