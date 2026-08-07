package com.stuypulse.robot.subsystems.handoff;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface HandoffConstants {
    public interface Handoff {
        double GEAR_RATIO = 3.0 / 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 4800.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 2200.0;
        double REVERSE_TIME = 2.0;
        double RPM_SOTM_TOLERANCE = 700.0;
        LoggedNetworkNumber HANDOFF_RPM = new LoggedNetworkNumber("/Tuning/Handoff/Target RPM", HANDOFF_MAX);

        double IS_EMPTY_AMPERAGE = 8; // TODO: update IS EMPTY VALUE

        double FORWARD_DUTY_CYCLE = 1.0;
        double REVERSE_DUTY_CYCLE = -1.0;

        LoggedNetworkNumber STALL_CURRENT_AMPS = new LoggedNetworkNumber(
                "/Tuning/Handoff/Stall Current Limit for Reverse", 30.0);
        double STALL_DEBOUNCE_SEC = 0.5;
    }
}
