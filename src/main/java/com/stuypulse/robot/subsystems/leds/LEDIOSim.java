/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.util.Color;

import com.ctre.phoenix6.signals.RGBWColor;

import org.littletonrobotics.junction.Logger;

public class LEDIOSim implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public boolean isInitialized = false;

  public LEDIOSim() {
    this.led = new AddressableLED(LEDConstants.Ports.LED_PORT);
    this.buffer = new AddressableLEDBuffer(LEDConstants.Settings.LED_LENGTH);

    led.setLength(buffer.getLength());
    led.setData(buffer);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.isConnected = true;
    inputs.supplyVoltage = Volts.of(12.0);
    inputs.fiveVRailVoltage = Volts.of(5.0);
    inputs.outputCurrentAmps = Amps.of(0.5);
    inputs.LEDTemperature = Celsius.of(25.0);
    inputs.hardwareFault = false;
    inputs.underVoltageFault = false;
  }

  public void applyOutputs(LEDIOOutputs outputs) {
    for (LEDPattern pattern : outputs.patterns) {
      RGBWColor color = pattern.color();

      int start = Math.max(pattern.start(), 0);
      int end = Math.min(pattern.end(), buffer.getLength() - 1);

      for (int i = start; i < end; i++) {
        buffer.setRGB(i, color.Red, color.Green, color.Blue);
      }
    }
  }

  public void periodicAfterScheduler(LEDIOOutputs outputs) {
    for (LEDPattern pattern : outputs.patterns) {
      RGBWColor color = pattern.color();
      Logger.recordOutput(
          "LEDs/Pattern" + pattern.start() + "-" + pattern.end(),
          new Color(color.Red, color.Green, color.Blue));
    }
  }
}
