package com.stuypulse.robot.subsystems.superstructure.shooter;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public Current shooterLeaderMotorSupplyCurrent = Amps.zero();
        public Current shooterLeaderMotorStatorCurrent = Amps.zero();
        public Temperature shooterLeaderMotorTemperature = Celsius.zero();
        public Angle shooterLeaderMotorPosition = Degrees.zero();
        public Voltage shooterLeaderMotorAppliedVoltage = Volts.zero();
        public AngularVelocity shooterLeaderMotorVelocity = DegreesPerSecond.zero();

        public Current shooterFollowerMotorSupplyCurrent = Amps.zero();
        public Current shooterFollowerMotorStatorCurrent = Amps.zero();
        public Temperature shooterFollowerMotorTemperature = Celsius.zero();
        public Angle shooterFollowerMotorPosition = Degrees.zero();
        public Voltage shooterFollowerMotorAppliedVoltage = Volts.zero();
        public AngularVelocity shooterFollowerMotorVelocity = DegreesPerSecond.zero();
    }

    class ShooterIOOutputs {
        public AngularVelocity shooterVelocity = RPM.zero();
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void applyOutputs(ShooterIOOutputs outputs) {
    }
}
