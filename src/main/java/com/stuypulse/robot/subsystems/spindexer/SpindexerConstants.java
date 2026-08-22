/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.stuypulse.robot.util.talonfx.TalonFXConfig;

public final class SpindexerConstants {
  private SpindexerConstants() {}

  public interface Settings {
    double FORWARD_DUTY_CYCLE = 1.0;
    double REVERSE_DUTY_CYCLE = -1.0;

    /* CONSTANTS */
    double GEAR_RATIO = 11.04 / 1.0;
  }

  public interface Motors {
    TalonFXConfig SPINDEXER_MOTOR_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimit(45)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withSensorToMechanismRatio(SpindexerConstants.Settings.GEAR_RATIO);
  }

  public interface Ports {
    int LEADER_MOTOR = 30;
    int FOLLOWER_MOTOR = 31; // TODO: follower port
  }
}
