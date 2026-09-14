/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOTalonFX extends IntakeIOBase {
  public IntakeIOTalonFX() {
    final TalonFX pivotMotor = new TalonFX(IntakeDeviceIds.PIVOT_MOTOR, GlobalSettings.RIO);
    final TalonFX rollerLeaderMotor =
        new TalonFX(IntakeDeviceIds.ROLLER_LEADER_MOTOR, GlobalSettings.RIO);
    final TalonFX rollerFollowerMotor =
        new TalonFX(IntakeDeviceIds.ROLLER_FOLLOWER_MOTOR, GlobalSettings.RIO);
    super(pivotMotor, rollerLeaderMotor, rollerFollowerMotor);
  }
}
