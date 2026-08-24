/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;

import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class IntakeIOSim extends IntakeIOBase {
  private final SystemSim<SingleJointedArmSim> pivotSim;
  private final TalonFXSimulation pivotMotor;

  private final SystemSim<FlywheelSim> rollerSim;
  private final TalonFXSimulation rollerLeaderMotor;
  private final TalonFXSimulation rollerFollowerMotor;

  public IntakeIOSim() {
    final double pivotGearRatio = IntakeSettings.PIVOT_GEAR_RATIO;
    final SystemSim<SingleJointedArmSim> pivotSim =
        SystemSim.of(
            new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                pivotGearRatio,
                IntakeSettings.PIVOT_MOI.in(KilogramSquareMeters),
                IntakeSettings.ARM_LENGTH.in(Meters),
                IntakeSettings.PIVOT_MIN_ANGLE.in(Radians),
                IntakeSettings.PIVOT_MAX_ANGLE.in(Radians),
                true,
                IntakeSettings.PIVOT_STOW_ANGLE.in(Radians)));
    final TalonFXSimulation pivotMotor =
        new TalonFXSimulation(IntakeDeviceIds.PIVOT_MOTOR, pivotGearRatio, pivotSim);

    final double rollerGearRatio = IntakeSettings.ROLLER_GEAR_RATIO;
    final SystemSim<FlywheelSim> rollerSim =
        SystemSim.of(
            new FlywheelSim(
                Models.flywheelFromPhysicalConstants(
                    DCMotor.getKrakenX60Foc(2),
                    IntakeSettings.ROLLER_MOI.in(KilogramSquareMeters),
                    rollerGearRatio),
                DCMotor.getKrakenX60Foc(2)));
    final TalonFXSimulation rollerLeaderMotor =
        new TalonFXSimulation(
            IntakeDeviceIds.ROLLER_LEADER_MOTOR, rollerGearRatio, rollerSim);
    final TalonFXSimulation rollerFollowerMotor =
        new TalonFXSimulation(
            IntakeDeviceIds.ROLLER_FOLLOWER_MOTOR, rollerGearRatio, rollerSim);

    super(pivotMotor, rollerLeaderMotor, rollerFollowerMotor);

    this.pivotSim = pivotSim;
    this.pivotMotor = pivotMotor;

    this.rollerSim = rollerSim;
    this.rollerLeaderMotor = rollerLeaderMotor;
    this.rollerFollowerMotor = rollerFollowerMotor;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    pivotSim.update(GlobalSettings.DT);
    pivotMotor.refresh();

    rollerSim.update(GlobalSettings.DT);
    rollerLeaderMotor.refresh();
    rollerFollowerMotor.refresh();

    super.updateInputs(inputs);
  }
}
