// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
    // ArmPathPlanner pathPlanner = new ArmPathPlanner(16.5, 20.5);
    // Point current = new Point(5, 0);
    // Point target = new Point(20, 10);
    
    // System.out.println(current);
    // System.out.println(pathPlanner.getArmAngles(current).get(0, 0) + ", " + pathPlanner.getArmAngles(current).get(1, 0));
    // current = pathPlanner.getNextTarget(current, target);

    // for (int i=0; i<20; i++) {
    //   System.out.println(current);
    //   System.out.println(pathPlanner.getArmAngles(current).get(0, 0) + ", " + pathPlanner.getArmAngles(current).get(1, 0));
    //   current = pathPlanner.getNextTarget(current, target);
    // }
  }
}
