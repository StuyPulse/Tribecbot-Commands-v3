/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.stuypulse.robot.util.talonfx.TalonFXConfig;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public interface HandoffConstants {
    public interface HandoffSettings {
        double GEAR_RATIO = 3.0 / 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 4800.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 2200.0;
        double REVERSE_TIME = 2.0;
        double RPM_SOTM_TOLERANCE = 700.0;
        LoggedNetworkNumber HANDOFF_RPM =
                new LoggedNetworkNumber("/Tuning/Handoff/Target RPM", HANDOFF_MAX);

        double FORWARD_DUTY_CYCLE = 1.0;
        double REVERSE_DUTY_CYCLE = -1.0;

        LoggedNetworkNumber HANDOFF_STALL_CURRENT_AMPS =
                new LoggedNetworkNumber("/Tuning/Handoff/Stall Current Limit for Reverse", 30.0);
        Time HANDOFF_STALL_DEBOUNCE = Seconds.of(0.5);

        // sim
        MomentOfInertia MOI = KilogramSquareMeters.of(0.0001);
    }

    public interface HandoffDeviceIds {
        int LEADER_MOTOR = 43;
        int FOLLOWER_MOTOR = 48;
    }

    public interface HandoffGains {
        double kP = 0.00015508;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.1728;
        double kV = 0.12;
        double kA = 0.00284;
    }

    public interface HandoffMotorConfigs {
        TalonFXConfig HANDOFF_MOTOR_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withSupplyCurrentLimit(Amps.of(80))
                        .withStatorCurrentLimitEnabled(false)
                        .withRampRate(0.25)
                        .withPIDConstants(HandoffGains.kP, HandoffGains.kI, HandoffGains.kD, 0)
                        .withFFConstants(HandoffGains.kS, HandoffGains.kV, HandoffGains.kA, 0)
                        .withSensorToMechanismRatio(HandoffSettings.GEAR_RATIO);
    }
}