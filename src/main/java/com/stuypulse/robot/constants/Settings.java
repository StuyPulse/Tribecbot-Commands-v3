/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import org.wpilib.framework.RobotBase;
import org.wpilib.units.measure.*;
import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use DogLog's tunables in order to have tunable values that we can edit
 * from external
 * dashboards.
 */
public interface Settings {
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

    public interface EnabledSubsystems {
        LoggedNetworkBoolean INTAKE = new LoggedNetworkBoolean("Enabled Subsystems/Intake", true);
        LoggedNetworkBoolean HANDOFF = new LoggedNetworkBoolean("Enabled Subsystems/Handoff", true);
        LoggedNetworkBoolean HOOD = new LoggedNetworkBoolean("Enabled Subsystems/Hood", true);
        LoggedNetworkBoolean SHOOTER = new LoggedNetworkBoolean("Enabled Subsystems/Shooter", true);
        LoggedNetworkBoolean TURRET = new LoggedNetworkBoolean("Enabled Subsystems/Turret", true);
    }

    public interface Intake {
        Angle PIVOT_STOW_ANGLE = Degrees.of(71.0);
        Angle PIVOT_DEPLOY_ANGLE = Degrees.of(-10.0);
        Angle PIVOT_DIGEST_ANGLE = Degrees.of(30);

        Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(5.0);

        Angle PIVOT_MAX_ANGLE = Degrees.of(76.4);
        Angle PIVOT_MIN_ANGLE = Degrees.of(-10.0);

        Angle THRESHOLD_TO_START_ROLLERS = Degrees.of(10.0);

        Angle ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Degrees.of(15.0);
        Voltage HOMING_VOLTAGE = Volts.of(3.0);

        Voltage PUSHDOWN_VOLTAGE = Volts.of(-3.0);
        Current PUSHDOWN_CURRENT_TELEOP = Amps.of(
                -75.0); // new SmartNumber("Intake/Pushdown Current", -65.0); //TODO: GET ACTUAL TYTY
        Current PUSHDOWN_CURRENT_AUTON = Amps.of(-80.0);

        double PIVOT_GEAR_RATIO = 32.0 / 20.0 * 64.0 / 18.0 * 60.0 / 8.0;

        Current PIVOT_STALL_CURRENT = Amps.of(0); // TODO: set value
        double PIVOT_STALL_DEBOUNCE = 1.0; // TODO: VERIFY

        double ROLLER_STALL_DEBOUNCE = 0.05; // TODO: VERIFY
        Current ROLLER_STALL_CURRENT = Amps.of(50.0);
    }
    public interface Handoff {
        public final double GEAR_RATIO = 3.0 / 1.0;
        
        Current HANDOFF_STALL_CURRENT = Amps.of(30); // TODO: set value
        double HANDOFF_STALL_DEBOUNCE_SEC = 0.5; // TODO: VERIFY
    }


    public interface Spindexer {
        double FORWARD_DUTY_CYCLE = 1.0;
        double ANTI_POPCORN_DUTY_CYCLE = 0.2;
        double REVERSE_DUTY_CYCLE = -1.0;
        double STOP_SPEED = 0.0;
        double REVERSE_TIME = 2.0;
        double ANTI_POPCORN_FREQ = 100;
        double ANTI_POPCORN_LENGTH = 10;

        double RPM_TOLERANCE = 800.0;
        double TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE = 1500.0;
        double STALL_CURRENT_LIMIT = 40.0; // random number as of 3/9

        double IS_EMPTY_AMPERAGE = 10; // TODO: update IS EMPTY VALUE

        /* CONSTANTS */
        double GEAR_RATIO = 11.04 / 1.0;
    }
}
