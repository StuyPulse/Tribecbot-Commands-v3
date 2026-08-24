/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodConstants.*;

import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  private static final Hood instance;

  static {
    switch (GlobalSettings.CURRENT_MODE) {
      case REAL -> instance = new Hood(new HoodIOTalonFX());

      case SIM -> instance = new Hood(new HoodIOSim());

      default -> instance = new Hood(new HoodIO() {});
    }
  }

  public static Hood getInstance() {
    return instance;
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;
  private final HoodIOOutputs outputs;

  @AutoLogOutput(key = "States/Intake")
  private HoodState state;

  private final Debouncer hoodStallingDebouncer;
  private final Debouncer hoodAtToleranceDebouncer;

  private Angle driverInput;

  private boolean atTolerance;

  private Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();
    outputs = new HoodIOOutputs();

    setState(HoodState.STOW);

    hoodStallingDebouncer =
        new Debouncer(HoodSettings.STALL_DEBOUNCE.in(Seconds), DebounceType.kBoth);
    hoodAtToleranceDebouncer = new Debouncer(HoodSettings.HOOD_AT_TOLERANCE_DEBOUNCE.in(Seconds), DebounceType.kBoth);

    this.atTolerance = false;
  }

  public enum HoodState {
    STOW,
    FERRY,
    MANUAL_OVERRIDE,
    KB,
    LEFT_CORNER,
    RIGHT_CORNER,
    INTERPOLATION,
    SOTM,
    FOTM,
    ANALOG,
    HOMING_UPPER,
    HOMING_LOWER,
    IDLE;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (!GlobalSettings.EnabledSubsystems.HOOD.get()) {
      stop();

      return;
    }

    switch (state) {
      case HOMING_UPPER -> {
        if (isStalling()) {
          io.seedHoodPosition(HoodSettings.MAX_FROM_HORIZON);
          setState(HoodState.STOW);
        } else {
          runVoltage(HoodSettings.HOOD_HOMING_VOLTAGE);
        }
      }
      case HOMING_LOWER -> {
        if (isStalling()) {
          io.seedHoodPosition(HoodSettings.MIN_FROM_HORIZON);
          setState(HoodState.STOW);
        } else {
          runVoltage(HoodSettings.HOOD_HOMING_VOLTAGE.unaryMinus());
        }
      }
      case STOW -> runPosition(HoodAngles.STOW);
      case FERRY -> runPosition(InterpolationCalculator.getInterpolatedFerryAngle());
      case MANUAL_OVERRIDE ->
          runPosition(
              Degrees.of(HoodAngles.MANUAL_OVERRIDE_DEG.get()));
      case KB -> runPosition(HoodAngles.KB);
      case LEFT_CORNER -> runPosition(HoodAngles.LEFT_CORNER);
      case RIGHT_CORNER -> runPosition(HoodAngles.RIGHT_CORNER);
      case INTERPOLATION -> runPosition(InterpolationCalculator.getInterpolatedShotAngle());
      case SOTM -> runPosition(SOTMCalculator.calculateHoodAngleSOTM());
      case FOTM -> runPosition(SOTMCalculator.calculateHoodAngleFOTM());
      case ANALOG -> runPosition(driverInput);
      case IDLE -> stop();
    }
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public boolean hoodReadyToShoot() {
    return hoodAtToleranceDebouncer.calculate(atTolerance);
  }

  private void stop() {
    outputs.outputMode = HoodIOOutputMode.STOP;
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public Angle getHoodAngle() {
    return inputs.hoodMotorPosition;
  }

  private void runPosition(Angle position) {
    outputs.outputMode = HoodIOOutputMode.POSITION;
    outputs.position = position;

    Angle error = inputs.hoodMotorPosition.minus(position);

    if (state == HoodState.SOTM || state == HoodState.FOTM) {
      atTolerance =
          error.abs(Degrees) < HoodSettings.HOOD_SOTM_TOLERANCE.in(Degrees);
    } else {
      atTolerance =
          error.abs(Degrees) < HoodSettings.HOOD_TOLERANCE.in(Degrees);
    }
  }

  private void runVoltage(Voltage voltage) {
    outputs.outputMode = HoodIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  private Angle hoodAnalogToInput(CommandGamepad gamepad) {
    double hoodMin = HoodAngles.MIN.in(Degrees);
    double hoodMax = HoodAngles.MAX.in(Degrees);

    return Degrees.of(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2));
  }

  private boolean isStalling() {
    return hoodStallingDebouncer.calculate(
        inputs.hoodMotorStatorCurrent.gt(
            HoodSettings.STALL_CURRENT_LIMIT));
  }

  public void setState(HoodState state) {
    this.state = state;
  }

  public Command runHomingUpper() {
    return run(coroutine -> setState(HoodState.HOMING_UPPER)).named("Run Homing Upper");
  }

  public Command runHomingLower() {
    return run(coroutine -> setState(HoodState.HOMING_LOWER)).named("Run Homing Lower");
  }

  public Command runStow() {
    return run(coroutine -> setState(HoodState.STOW)).named("Run Stow");
  }

  public Command runAnalog(CommandGamepad gamepad) {
    Command setAnalog = run(coroutine -> setState(HoodState.ANALOG)).named("Set analog");
    Command setInput = run(coroutine -> hoodAnalogToInput(gamepad)).named("Set input");
    return (setAnalog.andThen(setInput).named("Run analog"));
  }

  public Command seedRelativeEncoderAtUpperHardstop() {
    return run(coroutine ->
            io.seedHoodPosition(HoodSettings.MAX_FROM_HORIZON))
        .named("Hood Seed Relative Encoder at Upper Hardstop");
  }

  public Command seedRelativeEncoderAtLowerHardstop() {
    return run(coroutine ->
            io.seedHoodPosition(HoodSettings.MIN_FROM_HORIZON))
        .named("Hood Seed Relative Encoder at Lower Hardstop");
  }
}
