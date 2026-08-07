package com.stuypulse.robot.subsystems.intake;

import org.wpilib.units.measure.*;
import static org.wpilib.units.Units.*;

public interface IntakeConstants {
    public interface Intake {
        Angle PIVOT_STOW_ANGLE = Degrees.of(71.0);
        Angle PIVOT_DEPLOY_ANGLE = Degrees.of(-10.0);
        Angle PIVOT_DIGEST_ANGLE = Degrees.of(30);

        Angle PIVOT_ANGLE_TOLERANCE = Degrees.of(5.0);

        Angle PIVOT_MAX_ANGLE = Degrees.of(76.4);
        Angle PIVOT_MIN_ANGLE = Degrees.of(-10.0);

        Angle THRESHOLD_TO_START_ROLLERS = Degrees.of(10.0);

        Angle ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Degrees.of(15.0);
        Voltage HOMING_VOLTAGE = Volts.of(3.0);

        Voltage PUSHDOWN_VOLTAGE = Volts.of(-3.0);
        Current PUSHDOWN_CURRENT_TELEOP = Amps.of(
                -75.0); // new SmartNumber("Intake/Pushdown Current", -65.0); //TODO: GET ACTUAL TYTY
        Current PUSHDOWN_CURRENT_AUTON = Amps.of(-80.0);

        double PIVOT_GEAR_RATIO = 32.0 / 20.0 * 64.0 / 18.0 * 60.0 / 8.0;
        MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(0.138512); // found from onshape
        Distance PIVOT_ARM_LENGTH = Inches.of(17.522719); // estimate from onshape

        Current PIVOT_STALL_CURRENT = Amps.of(0); // TODO: set value
        double PIVOT_STALL_DEBOUNCE = 1.0; // TODO: VERIFY

        double ROLLER_STALL_DEBOUNCE = 0.05; // TODO: VERIFY
        Current ROLLER_STALL_CURRENT = Amps.of(50.0);

        MomentOfInertia ROLLER_MOI = KilogramSquareMeters.of(0.000358470114); // found on onshape
        double ROLLER_GEAR_RATIO = 22 / 36; // also from onshape
    }
}
