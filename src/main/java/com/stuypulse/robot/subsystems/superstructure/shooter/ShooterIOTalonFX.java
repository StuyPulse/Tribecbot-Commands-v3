/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.*;

public class ShooterIOTalonFX extends ShooterIOBase {
  public ShooterIOTalonFX() {
    final TalonFX shooterLeader = new TalonFX(ShooterDeviceIds.MOTOR_LEAD, GlobalSettings.RIO);
    final TalonFX shooterFollower =
        new TalonFX(ShooterDeviceIds.MOTOR_FOLLOW, GlobalSettings.RIO);
    super(shooterLeader, shooterFollower);
  }
}
