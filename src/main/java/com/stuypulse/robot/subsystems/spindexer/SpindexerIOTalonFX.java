/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.Ports;

public class SpindexerIOTalonFX extends SpindexerIOBase {
  public SpindexerIOTalonFX() {
    final TalonFX spindexerLeaderMotor =
        new TalonFX(SpindexerConstants.Ports.LEADER_MOTOR, Ports.RIO);
    final TalonFX spindexerFollowerMotor =
        new TalonFX(SpindexerConstants.Ports.FOLLOWER_MOTOR, Ports.RIO);
    super(spindexerLeaderMotor, spindexerFollowerMotor);
  }
}
