/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.commands.auton;

import org.wpilib.command3.Command;

public interface AutonomousRoutines {
    public static Command doNothingAuton() {
        return Command.sequence().named("Do Nothing");
    }
}
