/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class PathList {
  File frontLeft1File = new File("/home/lvuser/deploy/LeftSideHatchOne.pf1.csv");
  File frontLeftCenterFile = new File("/home/lvuser/deploy/FrontLeftHatchCenter.pf1.csv");
  File frontRightCenterFile = new File("/home/lvuser/deploy/FrontRightHatchCenter.pf1.csv");

  public static PathSetup LeftSideHatchOne;
  public static PathSetup FrontLeftHatchCenter;
  public static PathSetup FrontRightHatchCenter;
  public static PathSetup driveOffHab;
  private Waypoint[] driveOffHabWaypoints = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(4.95,0, 0), 
  };
  private double driveOffHabSpeed = 8;

  //remember that for all paths if the first point is at (0,0,0) for some reason the end y value is revesred in the coordinate plane
  //for example for a path from (x,y,h) to (0,0,0) a path that goes from (0,0,0) to (x,y,h) would look the same but for one you would 
  // be decreasing y units on the coordinate plane, while in the other you would be increasing y units
  public PathList() {
    driveOffHab = new PathSetup(driveOffHabWaypoints, driveOffHabSpeed, false);
    LeftSideHatchOne = new PathSetup(frontLeft1File, false);
    FrontLeftHatchCenter = new PathSetup(frontLeftCenterFile, false);
    FrontRightHatchCenter = new PathSetup(frontRightCenterFile, false);
  }
  public void resetAllPaths(){
    driveOffHab.resetPath();
  }
}
 
