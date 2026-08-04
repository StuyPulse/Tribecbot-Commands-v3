package com.stuypulse.robot.subsystems.handoff;



import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffIO.HandoffIOOutputs;
import org.wpilib.command3.*;

import static org.wpilib.units.Units.Amps;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;

public class Handoff extends Mechanism {
  private static final Handoff instance;

  static {
    switch (Settings.currentMode) {
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

  private final Debouncer handoffStallingDebouncer;

  private Handoff(HandoffIO io) {
    this.io = io;
    this.inputs = new HandoffIOInputsAutoLogged();
    this.outputs = new HandoffIOOutputs();

    this.handoffStallingDebouncer =
        new Debouncer(Settings.Handoff.HANDOFF_STALL_DEBOUNCE_SEC, DebounceType.kBoth);
  }

  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Handoff", inputs);
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Handoff/Duty Cycle Setpoint", outputs.handoffDutyCycle);
    io.applyOutputs(outputs);
  }

  private void runHandoffDutyCycle(double dutyCycle) {
    outputs.handoffDutyCycle = dutyCycle;
  }

  public boolean handoffStalling() {
    return handoffStallingDebouncer.calculate(
        inputs.motorLeadSupplyCurrent.abs(Amps) > Settings.Intake.PIVOT_STALL_CURRENT.in(Amps));
  }

  public Command runHandoff() {
    return run(
        coroutine -> {
          runHandoffDutyCycle(1.0);
        }).named("Handoff Forward");
  }

  public Command runHandoffReverse() {
    return run(
        coroutine -> {
          runHandoffDutyCycle(-1.0);
        }).named("Handoff Reverse");
  }

  public Command runHandoffStop() {
    return run(
        coroutine -> {
          runHandoffDutyCycle(0.0);
        }).named("Handoff Stop");
  }
}