/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * File containing tunable settings for every subsystem on the robot.
 *
 * <p>We use DogLog's tunables in order to have tunable values that we can edit from external
 * dashboards.
 */
public interface Settings {
  public static final Time DT = Milliseconds.of(20);
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum VisionMode {
    LIMELIGHT,
    PHOTON
  }

  public static final VisionMode currentVisionMode = VisionMode.PHOTON;

  public interface EnabledSubsystems {
    LoggedNetworkBoolean INTAKE = new LoggedNetworkBoolean("Enabled Subsystems/Intake", true);
    LoggedNetworkBoolean HANDOFF = new LoggedNetworkBoolean("Enabled Subsystems/Handoff", true);
    LoggedNetworkBoolean HOOD = new LoggedNetworkBoolean("Enabled Subsystems/Hood", true);
    LoggedNetworkBoolean SHOOTER = new LoggedNetworkBoolean("Enabled Subsystems/Shooter", true);
    LoggedNetworkBoolean TURRET = new LoggedNetworkBoolean("Enabled Subsystems/Turret", true);
    LoggedNetworkBoolean SPINDEXER = new LoggedNetworkBoolean("Enabled Subsystems/Spindexer", true);
    LoggedNetworkBoolean SWERVE = new LoggedNetworkBoolean("Enabled Subsystems/Swerve", true);
    LoggedNetworkBoolean VISION = new LoggedNetworkBoolean("Enabled Subsystems/Vision", true);
  }
}
