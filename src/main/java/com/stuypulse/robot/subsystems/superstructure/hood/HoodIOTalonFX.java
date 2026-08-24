/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.*;

public class HoodIOTalonFX extends HoodIOBase {
  public HoodIOTalonFX() {
    final TalonFX hoodMotor = new TalonFX(HoodDeviceIds.MOTOR, GlobalSettings.RIO);
    super(hoodMotor);
  }
}
