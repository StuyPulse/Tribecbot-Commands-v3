package com.stuypulse.robot.subsystems.handoff;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

public final class HandoffConstants {
    private HandoffConstants() {}

    public interface Settings {
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

    public interface Motors {
        TalonFXConfig HANDOFF_MOTOR_CONFIG = new TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
                .withSupplyCurrentLimit(80.0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25);
    }

    public interface Gains {}

    public interface Ports {
        int LEADER_MOTOR = 43;
        int FOLLOWER_MOTOR = 48;
    }
}
