/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Angle;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.*;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class HoodIOSim extends HoodIOBase {

  private final SystemSim<ElevatorSim> sim;

  private final TalonFXSimulation hoodMotor;

  private final PositionVoltage positionController;
  private final VoltageOut homingController;

  public HoodIOSim() {
    final SystemSim<ElevatorSim> sim =
        SystemSim.of(
            new ElevatorSim(
                Models.elevatorFromPhysicalConstants(
                    DCMotor.getKrakenX60(1),
                    HoodSettings.HOOD_MASS.in(Kilograms),
                    HoodSettings.DRUM_RADIUS.in(Meters),
                    HoodSettings.GEAR_RATIO),
                DCMotor.getKrakenX60(1),
                HoodSettings.MIN_HEIGHT.in(Meters),
                HoodSettings.MAX_HEIGHT.in(Meters),
                false,
                HoodSettings.MIN_HEIGHT.in(Meters),
                0.001),
            HoodSettings.DRUM_RADIUS);

    final TalonFXSimulation hoodMotor =
        new TalonFXSimulation(HoodDeviceIds.MOTOR, 1, sim);

    super(hoodMotor);

    this.hoodMotor = hoodMotor;
    this.sim = sim;

    positionController = new PositionVoltage(0).withEnableFOC(true);
    homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.update(GlobalSettings.DT);
    hoodMotor.refresh();

    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.outputMode) {
      case POSITION -> hoodMotor.setControl(positionController.withPosition(outputs.position));

      case VOLTAGE -> hoodMotor.setControl(homingController.withOutput(outputs.voltage));

      case STOP -> hoodMotor.stopMotor();
    }
  }

  @Override
  public void seedHoodPosition(Angle position) {
    hoodMotor.setPosition(position);
  }
}
