package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public Current hoodMotorSupplyCurrent = Amps.zero();
        public Current hoodMotorStatorCurrent = Amps.zero();
        public Temperature hoodMotorTemperature = Celsius.zero();
        public Angle hoodMotorPosition = Degrees.zero();
        public Voltage hoodMotorAppliedVoltage = Volts.zero();
        public AngularVelocity hoodMotorVelocity = DegreesPerSecond.zero();
    }

    public static enum HoodIOOutputMode {
        POSITION,
        VOLTAGE
    }

    public static class HoodIOOutputs {
        public HoodIOOutputMode outputMode = HoodIOOutputMode.POSITION;

        public Angle position = Degrees.zero();
        public Voltage voltage = Volts.zero();
    }

    public default void updateInputs(HoodIOInputs inputs) {
    }

    public default void applyOutputs(HoodIOOutputs ouptuts) {
    }

    public default void seedHoodPosition(Angle position) {
    }
}
