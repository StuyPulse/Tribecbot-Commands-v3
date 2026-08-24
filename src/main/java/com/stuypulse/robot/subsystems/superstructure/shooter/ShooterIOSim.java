/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterConstants.*;

import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class ShooterIOSim extends ShooterIOBase {

  // Sims
  private final SystemSim<FlywheelSim> flywheelSim;

  private final TalonFXSimulation shooterLeaderSim;
  private final TalonFXSimulation shooterFollowerSim;

  // Controllers
  private final VelocityTorqueCurrentFOC shooterLeaderController;
  private final Follower shooterFollowerController;

  public ShooterIOSim() {

    final SystemSim<FlywheelSim> flywheelSim =
        SystemSim.of(
            new FlywheelSim(
                Models.flywheelFromPhysicalConstants(
                    DCMotor.getKrakenX44(2),
                    ShooterSettings.FLYWHEEL_MOI.in(KilogramSquareMeters),
                    ShooterSettings.GEAR_RATIO),
                DCMotor.getKrakenX44(2),
                ShooterSettings.GEAR_RATIO));

    final TalonFXSimulation shooterLeaderSim =
        new TalonFXSimulation(
            ShooterDeviceIds.MOTOR_LEAD,
            ShooterSettings.GEAR_RATIO,
            flywheelSim);
    final TalonFXSimulation shooterFollowerSim =
        new TalonFXSimulation(
            ShooterDeviceIds.MOTOR_FOLLOW,
            ShooterSettings.GEAR_RATIO,
            flywheelSim);

    final VelocityTorqueCurrentFOC shooterLeaderController = new VelocityTorqueCurrentFOC(0);

    final Follower shooterFollowerController =
        new Follower(shooterLeaderSim.getDeviceID(), MotorAlignmentValue.Opposed);
    shooterFollowerSim.setControl(shooterFollowerController);

    super(shooterLeaderSim, shooterFollowerSim);

    this.shooterFollowerSim = shooterFollowerSim;
    this.shooterLeaderSim = shooterLeaderSim;
    this.flywheelSim = flywheelSim;
    this.shooterLeaderController = shooterLeaderController;
    this.shooterFollowerController = shooterFollowerController;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(GlobalSettings.DT);
    shooterLeaderSim.refresh();
    shooterFollowerSim.refresh();

    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    shooterLeaderSim.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
  }
}
