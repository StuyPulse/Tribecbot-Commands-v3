/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import org.wpilib.util.Color;

import com.ctre.phoenix6.signals.RGBWColor;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDPattern;
import com.stuypulse.robot.util.FullSubsystem;

import org.littletonrobotics.junction.Logger;

public class LEDController extends FullSubsystem {
  private static final LEDController instance; // LED instance

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new LEDController(new LEDIOCANdle() {});

      case SIM -> instance = new LEDController(new LEDIOSim() {});

      default -> instance = new LEDController(new LEDIO() {});
    }
  }

  public static LEDController getInstance() { // getter
    return instance;
  }

  // IO fields
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs;
  private final LEDIOOutputs outputs;

  // CANdle

  public LEDController(LEDIO io) { // might have to be private
    this.io = io;
    this.inputs = new LEDIOInputsAutoLogged();
    this.outputs = new LEDIOOutputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);

    for (LEDPattern pattern : outputs.patterns) {
      RGBWColor color = pattern.color();
      Logger.recordOutput(
          "LEDs/Pattern" + pattern.start() + "-" + pattern.end(),
          new Color(color.Red, color.Green, color.Blue));
    }
  }
}
