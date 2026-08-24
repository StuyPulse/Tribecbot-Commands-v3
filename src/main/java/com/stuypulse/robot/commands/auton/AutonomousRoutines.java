/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton;

import org.wpilib.command3.Command;

public interface AutonomousRoutines {
  public static Command doNothingAuton() {
    return Command.sequence().named("Do Nothing");
  }
}
