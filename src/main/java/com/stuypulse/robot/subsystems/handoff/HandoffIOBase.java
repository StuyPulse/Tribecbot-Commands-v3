/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import org.wpilib.units.measure.*;

import com.stuypulse.robot.subsystems.handoff.HandoffConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public abstract class HandoffIOBase implements HandoffIO {
  private final TalonFX handoffLeaderMotor;
  private final TalonFX handoffFollowerMotor;

  private final DutyCycleOut controller;
  private final Follower follower;

  private final StatusSignal<Current> motorLeadSupplyCurrent;
  private final StatusSignal<Current> motorLeadStatorCurrent;
  private final StatusSignal<Temperature> motorLeadTemperature;
  private final StatusSignal<AngularVelocity> motorLeadVelocity;
  private final StatusSignal<Voltage> motorLeadAppliedVoltage;

  private final StatusSignal<Current> motorFollowSupplyCurrent;
  private final StatusSignal<Current> motorFollowStatorCurrent;
  private final StatusSignal<Temperature> motorFollowTemperature;
  private final StatusSignal<AngularVelocity> motorFollowVelocity;
  private final StatusSignal<Voltage> motorFollowAppliedVoltage;

  public HandoffIOBase(TalonFX handoffLeaderMotor, TalonFX handoffFollowerMotor) {
    this.handoffLeaderMotor = handoffLeaderMotor;
    this.handoffFollowerMotor = handoffFollowerMotor;

    HandoffMotorConfigs.HANDOFF_MOTOR_CONFIG.configure(handoffLeaderMotor);
    HandoffMotorConfigs.HANDOFF_MOTOR_CONFIG.configure(handoffFollowerMotor);

    controller = new DutyCycleOut(0).withEnableFOC(true);
    follower = new Follower(handoffLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);

    handoffFollowerMotor.setControl(follower);

    motorLeadSupplyCurrent = handoffLeaderMotor.getSupplyCurrent();
    motorLeadStatorCurrent = handoffLeaderMotor.getStatorCurrent();
    motorLeadTemperature = handoffLeaderMotor.getDeviceTemp();
    motorLeadVelocity = handoffLeaderMotor.getVelocity();
    motorLeadAppliedVoltage = handoffLeaderMotor.getMotorVoltage();

    motorFollowSupplyCurrent = handoffLeaderMotor.getSupplyCurrent();
    motorFollowStatorCurrent = handoffLeaderMotor.getStatorCurrent();
    motorFollowTemperature = handoffLeaderMotor.getDeviceTemp();
    motorFollowVelocity = handoffLeaderMotor.getVelocity();
    motorFollowAppliedVoltage = handoffLeaderMotor.getMotorVoltage();
  }

  @Override
  public void updateInputs(HandoffIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorLeadSupplyCurrent,
        motorLeadStatorCurrent,
        motorLeadTemperature,
        motorLeadVelocity,
        motorLeadAppliedVoltage,
        motorFollowSupplyCurrent,
        motorFollowStatorCurrent,
        motorFollowTemperature,
        motorFollowVelocity,
        motorFollowAppliedVoltage);

    inputs.motorLeadSupplyCurrent = motorLeadSupplyCurrent.getValue();
    inputs.motorLeadStatorCurrent = motorLeadStatorCurrent.getValue();
    inputs.motorLeadTemperature = motorLeadTemperature.getValue();
    inputs.motorLeadVelocity = motorLeadVelocity.getValue();
    inputs.motorLeadAppliedVoltage = motorLeadAppliedVoltage.getValue();

    inputs.motorFollowSupplyCurrent = motorFollowSupplyCurrent.getValue();
    inputs.motorFollowStatorCurrent = motorFollowStatorCurrent.getValue();
    inputs.motorFollowTemperature = motorFollowTemperature.getValue();
    inputs.motorFollowVelocity = motorFollowVelocity.getValue();
    inputs.motorFollowAppliedVoltage = motorFollowAppliedVoltage.getValue();
  }

  @Override
  public void applyOutputs(HandoffIOOutputs outputs) {
    switch (outputs.handoffMode) {
      case DUTY_CYCLE ->
          handoffLeaderMotor.setControl(controller.withOutput(outputs.handoffDutyCycle));
      case STOP -> {
        handoffLeaderMotor.stopMotor();
        handoffFollowerMotor.stopMotor();
        handoffFollowerMotor.setControl(follower);
      }
    }
  }
}
