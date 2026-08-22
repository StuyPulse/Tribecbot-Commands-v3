package com.stuypulse.robot.subsystems.handoff;

import org.wpilib.units.measure.*;
import static org.wpilib.units.Units.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.talonfx.TalonFXConfig;

public final class HandoffConstants {
    private HandoffConstants() {}

    public interface Settings {
        double GEAR_RATIO = 3.0 / 1.0;

        double FORWARD_DUTY_CYCLE = 1.0;
        double REVERSE_DUTY_CYCLE = -1.0;

        LoggedNetworkNumber STALL_CURRENT_AMPS = new LoggedNetworkNumber(
                "/Tuning/Handoff/Stall Current Limit for Reverse", 30.0);
        Time STALL_DEBOUNCE = Seconds.of(0.5);
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
