/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import static org.wpilib.units.Units.KilogramSquareMeters;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerConstants.*;

import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class SpindexerIOSim extends SpindexerIOBase {
  private final SystemSim<FlywheelSim> spindexerSim;

  private final TalonFXSimulation spindexerLeaderMotor;
  private final TalonFXSimulation spindexerFollowerMotor;

  public SpindexerIOSim() {
    final double gearing = SpindexerSettings.GEAR_RATIO;
    final SystemSim<FlywheelSim> spindexerSim =
        SystemSim.of(
            new FlywheelSim(
                Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), SpindexerSettings.SPINDEXER_MOI.in(KilogramSquareMeters), gearing),
                DCMotor.getKrakenX60(1),
                0.01));

    final TalonFXSimulation spindexerLeaderMotor =
        new TalonFXSimulation(SpindexerDeviceIds.LEADER_MOTOR, gearing, spindexerSim);
    final TalonFXSimulation spindexerFollowerMotor =
        new TalonFXSimulation(SpindexerDeviceIds.FOLLOWER_MOTOR, gearing, spindexerSim);

    super(spindexerLeaderMotor, spindexerFollowerMotor);

    this.spindexerSim = spindexerSim;
    this.spindexerLeaderMotor = spindexerLeaderMotor;
    this.spindexerFollowerMotor = spindexerFollowerMotor;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    spindexerSim.update(GlobalSettings.DT);

    spindexerLeaderMotor.refresh();
    spindexerFollowerMotor.refresh();

    super.updateInputs(inputs);
  }
}
