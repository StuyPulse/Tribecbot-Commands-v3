/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.*;

public class HandoffIOTalonFX extends HandoffIOBase {
  public HandoffIOTalonFX() {
    final TalonFX handoffLeaderMotor = new TalonFX(HandoffDeviceIds.LEADER_MOTOR, GlobalSettings.RIO);
    final TalonFX handoffFollowerMotor =
        new TalonFX(HandoffDeviceIds.FOLLOWER_MOTOR, GlobalSettings.RIO);
    super(handoffLeaderMotor, handoffFollowerMotor);
  }
}
