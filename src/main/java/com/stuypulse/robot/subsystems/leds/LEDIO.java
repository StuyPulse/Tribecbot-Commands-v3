package com.stuypulse.robot.subsystems.leds;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.RGBWColor;

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

    public static class LEDIOOutputs {
        public RGBWColor color;
    }

    public default void updateInputs(LEDIOInputs inputs) {
    }

    public default void applyOutputs(LEDIOOutputs outputs) {
    }

    default void setControl(ControlRequest request) {
    }

    default void clearAllAnimations() {
    }
}
