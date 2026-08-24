/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;

public class TurretIOTalonFX extends TurretIOBase {
  public TurretIOTalonFX() {
    final TalonFX turretMotor = new TalonFX(TurretDeviceIds.MOTOR, GlobalSettings.RIO);
    final CANcoder encoder17t = new CANcoder(TurretDeviceIds.ENCODER17T, GlobalSettings.RIO);
    final CANcoder encoder18t = new CANcoder(TurretDeviceIds.ENCODER18T, GlobalSettings.RIO);
    super(turretMotor, encoder17t, encoder18t);
  }
}
