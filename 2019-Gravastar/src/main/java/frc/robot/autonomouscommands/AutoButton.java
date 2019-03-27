package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.tools.PID;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.util.UncleanStatusException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class AutoButton extends Command {
  
  PID turnController = new PID(0, 0, 0);

  private double forwardSpeed = 0.4;

	private double kp = 0.025;
	private double ki = 0.00025;
  private double kd = 0.000025;
  
  public PID orientation;

  VisionCamera jevois = new VisionCamera(Robot.jevois1);
  private double startAngle = jevois.angle;
  private String json = jevois.sanatizedString;

  public AutoButton() {

    orientation = new PID(kp, ki, kd);

  }


  @Override
  protected void initialize() {

    orientation.setSetPoint(0);
  
  }

  @Override
  protected void execute() {
  
      if (startAngle != 0){

        orientation.updatePID(startAngle);
        RobotMap.leftDriveLead.set(ControlMode.PercentOutput, -forwardSpeed - orientation.getResult());
        RobotMap.rightDriveLead.set(ControlMode.PercentOutput, -forwardSpeed + orientation.getResult());
        
          if (startAngle == -100 || json.isBlank() || startAngle == 0){
            RobotMap.leftDriveLead.set(ControlMode.PercentOutput, forwardSpeed);
            RobotMap.rightDriveLead.set(ControlMode.PercentOutput, forwardSpeed);
          
        }
      }
    }

  @Override
  protected boolean isFinished() {

    if (startAngle == 0){
     
      return true;
    
    }

   else return false;
  
  }

  @Override
  protected void end() {

    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
		RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);

  }

  @Override
  protected void interrupted() {
  }
}
