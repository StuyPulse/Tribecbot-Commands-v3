/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.wpilib.hardware.led.LEDPattern;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.RGBWColor;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public boolean isConnected = false;
    public Voltage supplyVoltage = Volts.zero();
    public Voltage fiveVRailVoltage = Volts.zero();
    public Current outputCurrentAmps = Amps.zero();
    public Temperature LEDTemperature = Celsius.zero();
    public boolean hardwareFault = false;
    public boolean underVoltageFault = false;
  }

  public record LEDPattern(int start, int end, RGBWColor color) {}

  public static class LEDIOOutputs {
    public RGBWColor color;

    public final List<LEDPattern> patterns = new ArrayList<>();
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void applyOutputs(LEDIOOutputs outputs) {}

  default void setControl(ControlRequest request) {}

  default void clearAllAnimations() {}
}
