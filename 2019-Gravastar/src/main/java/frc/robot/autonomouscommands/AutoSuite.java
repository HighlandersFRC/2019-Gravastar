package frc.robot.autonomouscommands;

import frc.robot.RobotMap;

public class AutoSuite {
    private PurePursuitController leftSideHatchOne = new PurePursuitController(RobotMap.universalPathList.LeftSideHatchOne, 1.9, 2.25, 0.05);
    private PurePursuitController frontLeftHatchCenter = new PurePursuitController(RobotMap.universalPathList.FrontLeftHatchCenter, 1.9, 2.25, 0.05);
    public AutoSuite() {
    }
    public void startAutoCommands() {
        leftSideHatchOne.start();
        //frontLeftHatchCenter.start();
    }
    public void endAutoCommands(){
        leftSideHatchOne.cancel();
        //frontLeftHatchCenter.cancel();
    }

}