/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;

import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class TurretIOSim extends TurretIOBase {

  private SystemSim<DCMotorSim> sim;
  private PositionVoltage controller;
  private TalonFXSimulation simMotor;

  public TurretIOSim() {
    final SystemSim<DCMotorSim> sim =
        SystemSim.of(
            new DCMotorSim(
                Models.singleJointedArmFromPhysicalConstants(
                    DCMotor.getKrakenX60(1),
                    TurretSettings.TURRET_MOI.in(KilogramSquareMeters),
                    TurretSettings.GEAR_RATIO_MOTOR_TO_MECH),
                DCMotor.getKrakenX60(1),
                2.8));

    final TalonFXSimulation simMotor =
        new TalonFXSimulation(
            TurretDeviceIds.MOTOR,
            TurretSettings.GEAR_RATIO_MOTOR_TO_MECH,
            sim);

    controller = new PositionVoltage(0).withEnableFOC(true);

    super(simMotor, null, null);

    this.simMotor = simMotor;
    this.sim = sim;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.update(GlobalSettings.DT);
    simMotor.refresh();

    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    simMotor.setControl(controller.withPosition(outputs.turretPosition));
  }
}
