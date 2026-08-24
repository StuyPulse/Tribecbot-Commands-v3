/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerConstants.*;

public class SpindexerIOTalonFX extends SpindexerIOBase {
  public SpindexerIOTalonFX() {
    final TalonFX spindexerLeaderMotor =
        new TalonFX(SpindexerDeviceIds.LEADER_MOTOR, GlobalSettings.RIO);
    final TalonFX spindexerFollowerMotor =
        new TalonFX(SpindexerDeviceIds.FOLLOWER_MOTOR, GlobalSettings.RIO);
    super(spindexerLeaderMotor, spindexerFollowerMotor);
  }
}
