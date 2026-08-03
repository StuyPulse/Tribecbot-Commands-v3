package com.stuypulse.robot.subsystems.superstructure.turret;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public Current turretMotorSupplyCurrent = Amps.zero();
        public Current turretMotorStatorCurrent = Amps.zero();
        public Temperature turretMotorTemperature = Celsius.zero();
        public Angle turretMotorPosition = Degrees.zero();
        public Voltage turretMotorAppliedVoltage = Volts.zero();
        public AngularVelocity turretMotorVelocity = DegreesPerSecond.zero();
    }

    public static class TurretIOOutputs {
        public Angle turretPosition = Degrees.zero();
    }

    public default void updateInputs(TurretIOInputs inputs) {
    }

    public default void applyOutputs(TurretIOOutputs outputs) {
    }
}
