/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.constants.DriverConstants.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import com.stuypulse.robot.commands.auton.AutonomousRoutines;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class RobotContainer {

  // Gamepads
  public static final CommandNiDsXboxController driver =
      new CommandNiDsXboxController(DriverSettings.INDEX);

  // Subsystem

  // Autons
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Robot container

  public RobotContainer() {
    configureLogging();
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /***************/
  /*** LOGGING ***/
  /***************/

  private void configureLogging() {
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withNtTunables(true).withLogExtras(true));
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {}

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {}

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.setDefaultOption("Do Nothing", AutonomousRoutines.doNothingAuton());

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return The command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
