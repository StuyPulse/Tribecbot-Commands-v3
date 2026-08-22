/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.*;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.handoff.HandoffConstants.*;

import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputMode;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Handoff extends FullSubsystem {
  private static final Handoff instance;

  static {
    switch (GlobalSettings.CURRENT_MODE) {
      case REAL -> instance = new Handoff(new HandoffIOTalonFX());

      case SIM -> instance = new Handoff(new HandoffIOSim());

      default -> instance = new Handoff(new HandoffIO() {});
    }
  }

  public static Handoff getInstance() {
    return instance;
  }

  private final HandoffIO io;
  private final HandoffIOInputsAutoLogged inputs;
  private final HandoffIOOutputs outputs;

  @AutoLogOutput(key = "States/Handoff")
  private HandoffState state;

  private final Debouncer handoffStallingDebouncer;

  private Handoff(HandoffIO io) {
    this.io = io;
    this.inputs = new HandoffIOInputsAutoLogged();
    this.outputs = new HandoffIOOutputs();

    setState(HandoffState.STOP);

    this.handoffStallingDebouncer =
        new Debouncer(HandoffSettings.HANDOFF_STALL_DEBOUNCE.in(Seconds), DebounceType.kBoth);
  }

  public enum HandoffState {
    FORWARD,
    REVERSE,
    STOP
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Handoff", inputs);

    if (!GlobalSettings.EnabledSubsystems.HANDOFF.get()) {
      stopHandoff();

      return;
    }

    switch (state) {
      case FORWARD -> runHandoffDutyCycle(HandoffSettings.FORWARD_DUTY_CYCLE);
      case REVERSE -> runHandoffDutyCycle(HandoffSettings.REVERSE_DUTY_CYCLE);
      case STOP -> stopHandoff();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runHandoffDutyCycle(double dutyCycle) {
    outputs.handoffMode = HandoffIOOutputMode.DUTY_CYCLE;
    outputs.handoffDutyCycle = dutyCycle;
  }

  private void stopHandoff() {
    outputs.handoffMode = HandoffIOOutputMode.STOP;
  }

  public boolean isHandoffStalling() {
    return handoffStallingDebouncer.calculate(
        inputs.motorLeadSupplyCurrent.abs(Amps)
            > HandoffSettings.HANDOFF_STALL_CURRENT_AMPS.get());
  }

  public HandoffState getState() {
    return state;
  }

  private void setState(HandoffState state) {
    this.state = state;
  }

  public void setStateCommand(HandoffState state) {
    setState(state);
  }

  public Command runHandoffForward() {
    return run(coroutine -> setState(HandoffState.FORWARD)).named("Handoff Forward");
  }

  public Command runHandoffReverse() {
    return run(coroutine -> setState(HandoffState.REVERSE)).named("Handoff Reverse");
  }

  public Command stopHandoffCommand() {
    return run(coroutine -> setState(HandoffState.STOP)).named("Handoff Stop");
  }
}
