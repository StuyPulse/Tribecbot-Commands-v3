package com.stuypulse.robot.subsystems.spindexer;

public interface SpindexerConstants {
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
