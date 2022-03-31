/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import org.wpilib.framework.RobotBase;

/**
 * Main Class
 *
 * This is the main class that instantiates the robot code.
 * There is no need to edit this file, and it should not be edited unless you know what you are doing.
 */
public final class Main {
    private Main() {}

    /**
     * The main method that starts the robot code. This should not be edited unless you know what
     * you are doing.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot.class);
    }
}
