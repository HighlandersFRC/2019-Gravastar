package frc.robot.autonomouscommands;

import frc.robot.RobotMap;

public class AutoSuite {
    private PurePursuitController leftSideHatchOne = new PurePursuitController(RobotMap.universalPathList.LeftSideHatchOne, 1.9, 2.25, 0.05);
    private PurePursuitController frontLeftHatchCenter = new PurePursuitController(RobotMap.universalPathList.FrontLeftHatchCenter, 1.9, 2.25, 0.05);
    private PurePursuitController frontRightHatchCenter = new PurePursuitController(RobotMap.universalPathList.FrontRightHatchCenter, 1.9, 2.25, 0.05);
    private PurePursuitController driveOffHab = new PurePursuitController(RobotMap.universalPathList.driveOffHab, 1.9, 2.25, 0.05);
    private CascadingPIDTurn turn = new CascadingPIDTurn(20.0,0.08,0.00085,0.00006);
    //private CascadingDriveStraightPID straight = new CascadingDriveStraightPID( 2, 1);
    public AutoSuite() {
    }
    public void startAutoCommands() {
        //straight.start();
        //leftSideHatchOne.start();
        //frontLeftHatchCenter.start();
        //frontRightHatchCenter.start();
        //driveOffHab.start();
       turn.start();
    }
    public void endAutoCommands(){
        //straight.cancel();
        //leftSideHatchOne.cancel();
        //frontLeftHatchCenter.cancel();
        //driveOffHab.cancel();
        //frontRightHatchCenter.cancel();
        turn.start();
    }

}