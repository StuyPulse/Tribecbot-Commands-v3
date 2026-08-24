/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static org.wpilib.units.Units.KilogramSquareMeters;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.*;

import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;


public class HandoffIOSim extends HandoffIOBase {
  private final SystemSim<FlywheelSim> handoffSim;

  private final TalonFXSimulation handoffLeaderMotor;
  private final TalonFXSimulation handoffFollowerMotor;

  public HandoffIOSim() {
    final SystemSim<FlywheelSim> handoffSim =
        SystemSim.of(
            new FlywheelSim(
                Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), HandoffSettings.MOI.in(KilogramSquareMeters), HandoffSettings.GEAR_RATIO),
                DCMotor.getKrakenX60(1),
                0.01));

    final TalonFXSimulation handoffLeaderMotor =
        new TalonFXSimulation(
            HandoffDeviceIds.LEADER_MOTOR, HandoffSettings.GEAR_RATIO, handoffSim);
    final TalonFXSimulation handoffFollowerMotor =
        new TalonFXSimulation(
            HandoffDeviceIds.FOLLOWER_MOTOR,
            HandoffSettings.GEAR_RATIO,
            handoffSim);

    super(handoffLeaderMotor, handoffFollowerMotor);

    this.handoffSim = handoffSim;
    this.handoffLeaderMotor = handoffLeaderMotor;
    this.handoffFollowerMotor = handoffFollowerMotor;
  }

  @Override
  public void updateInputs(HandoffIOInputs inputs) {
    handoffSim.update(GlobalSettings.DT);

    handoffLeaderMotor.refresh();
    handoffFollowerMotor.refresh();

    super.updateInputs(inputs);
  }
}
