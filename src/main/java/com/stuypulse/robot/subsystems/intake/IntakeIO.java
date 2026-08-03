package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public Current pivotMotorSupplyCurrent = Amps.zero();
        public Current pivotMotorStatorCurrent = Amps.zero();
        public Temperature pivotMotorTemperature = Celsius.zero();
        public Angle pivotMotorPosition = Degrees.zero();
        public Voltage pivotMotorAppliedVoltage = Volts.zero();
        public AngularVelocity pivotMotorVelocity = DegreesPerSecond.zero();

        public Current rollerLeaderMotorSupplyCurrent = Amps.zero();
        public Current rollerLeaderMotorStatorCurrent = Amps.zero();
        public Temperature rollerLeaderMotorTemperature = Celsius.zero();
        public Angle rollerLeaderMotorPosition = Degrees.zero();
        public Voltage rollerLeaderMotorAppliedVoltage = Volts.zero();
        public AngularVelocity rollerLeaderMotorVelocity = DegreesPerSecond.zero();

        public Current rollerFollowerMotorSupplyCurrent = Amps.zero();
        public Current rollerFollowerMotorStatorCurrent = Amps.zero();
        public Temperature rollerFollowerMotorTemperature = Celsius.zero();
        public Angle rollerFollowerMotorPosition = Degrees.zero();
        public Voltage rollerFollowerMotorAppliedVoltage = Volts.zero();
        public AngularVelocity rollerFollowerMotorVelocity = DegreesPerSecond.zero();
    }

    public static enum PivotIOOutputMode {
        POSITION,
        TORQUE_CURRENT,
        VOLTAGE
    }

    public static class IntakeIOOutputs {
        public PivotIOOutputMode pivotOutputMode = PivotIOOutputMode.POSITION;
        public Angle pivotPosition = Degrees.zero();
        public Current pivotTorqueCurrent = Amps.zero();
        public Voltage pivotVoltage = Volts.zero();

        public double rollerDutyCycle = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void applyOutputs(IntakeIOOutputs outputs) {
    }

    public default void seedPivotPosition(Angle position) {
    }
}
