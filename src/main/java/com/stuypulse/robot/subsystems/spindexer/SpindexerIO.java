package com.stuypulse.robot.subsystems.spindexer;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public Current spindexerLeaderMotorSupplyCurrent = Amps.zero();
        public Current spindexerLeaderMotorStatorCurrent = Amps.zero();
        public Temperature spindexerLeaderMotorTemperature = Celsius.zero();
        public Angle spindexerLeaderMotorPosition = Degrees.zero();
        public Voltage spindexerLeaderMotorAppliedVoltage = Volts.zero();
        public AngularVelocity spindexerLeaderMotorVelocity = DegreesPerSecond.zero();

        public Current spindexerFollowerMotorSupplyCurrent = Amps.zero();
        public Current spindexerFollowerMotorStatorCurrent = Amps.zero();
        public Temperature spindexerFollowerMotorTemperature = Celsius.zero();
        public Angle spindexerFollowerMotorPosition = Degrees.zero();
        public Voltage spindexerFollowerMotorAppliedVoltage = Volts.zero();
        public AngularVelocity spindexerFollowerMotorVelocity = DegreesPerSecond.zero();
    }

    public static class SpindexerIOOutputs {
        public double spindexerLeaderDutyCycle = 0;
    }

    public default void updateInputs(SpindexerIOInputs inputs) {
    }

    public default void applyOutputs(SpindexerIOOutputs outputs) {
    }
}
