/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import org.wpilib.framework.RobotBase;

/**
 * Main Class
 *
 * <p>This is the main class that instantiates the robot code. There is no need to edit this file,
 * and it should not be edited unless you know what you are doing.
 */
public final class Main {
  private Main() {}

  /**
   * The main method that starts the robot code. This should not be edited unless you know what you
   * are doing.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot.class);
  }
}
