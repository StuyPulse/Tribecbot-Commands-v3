/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

import com.stuypulse.robot.util.FullSubsystem;

import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot Class
 *
 * <p>This is the main class for robot code, instantiated in {@link com.stuypulse.robot.Main} It
 * extends TimedRobot, meaning that the methods in this class are called automatically during
 * specific states of the robot.
 */
public class Robot extends LoggedRobot {
  /**
   * Checks the alliance the robot is on
   *
   * @return true if the robot is on the blue alliance, false if the robot is on the red alliance,
   *     and false if alliance cannot be determined.
   */
  public static boolean isBlue() {
    final Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.BLUE;
    }
    return false;
  }

  private final RobotContainer robot;
  private final Scheduler defaultScheduler;
  private Command auto;

  public Robot() {
    robot = new RobotContainer();
    defaultScheduler = Scheduler.getDefault();

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
  }

  /*************************/
  /*** ROBOT SCHEDULEING ***/
  /*************************/

  @Override
  public void robotPeriodic() {
    defaultScheduler.run();

    FullSubsystem.runAllPeriodicAfterScheduler();
  }

  /*********************/
  /*** DISABLED MODE ***/
  /*********************/

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /***********************/
  /*** AUTONOMOUS MODE ***/
  /***********************/

  @Override
  public void autonomousInit() {
    auto = robot.getAutonomousCommand();

    if (auto != null) {
      defaultScheduler.schedule(auto);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  /*******************/
  /*** TELEOP MODE ***/
  /*******************/

  @Override
  public void teleopInit() {
    if (auto != null) {
      defaultScheduler.cancel(auto);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  /*************************/
  /*** UTILITY/TEST MODE ***/
  /*************************/

  @Override
  public void utilityInit() {
    defaultScheduler.cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}
}
