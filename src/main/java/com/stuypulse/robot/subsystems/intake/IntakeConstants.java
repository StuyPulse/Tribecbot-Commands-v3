package com.stuypulse.robot.subsystems.intake;

import org.wpilib.units.measure.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class IntakeConstants {
    private IntakeConstants() {}

    public interface Settings {
        public interface Pivot {
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
        }

        public static interface Roller {
            double ROLLER_STALL_DEBOUNCE = 0.05; // TODO: VERIFY
            Current ROLLER_STALL_CURRENT = Amps.of(50.0);

            MomentOfInertia ROLLER_MOI = KilogramSquareMeters.of(0.000358470114); // found on onshape
            double ROLLER_GEAR_RATIO = 22 / 36; // also from onshape
        }
    }

    public static interface Gains {
        LoggedNetworkNumber kP = new LoggedNetworkNumber("Intake/Pivot/Gains/kP", 125.0);
        LoggedNetworkNumber kI = new LoggedNetworkNumber("Intake/Pivot/Gains/kI", 0.0);
        LoggedNetworkNumber kD = new LoggedNetworkNumber("Intake/Pivot/Gains/kD", 10.0);

        LoggedNetworkNumber kS = new LoggedNetworkNumber("Intake/Pivot/Gains/kS", 0.0);
        LoggedNetworkNumber kV = new LoggedNetworkNumber("Intake/Pivot/Gains/kV", 0.12);
        LoggedNetworkNumber kA = new LoggedNetworkNumber("Intake/Pivot/Gains/kA", 0.0);

        double kG = 0.5;
    }

    public interface Motors {
        TalonFXConfig PIVOT_CONFIG = new TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
                .withSupplyCurrentLimit(10.0) // was 60 on practice day
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25)
                .withPIDConstants(
                        Gains.kP.get(),
                        Gains.kI.get(),
                        Gains.kD.get(),
                        0)
                .withFFConstants(
                        Gains.kS.get(),
                        Gains.kV.get(),
                        Gains.kA.get(),
                        Gains.kG,
                        0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withSensorToMechanismRatio(IntakeConstants.Settings.Pivot.PIVOT_GEAR_RATIO);

        TalonFXConfig ROLLER_CONFIG = new TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
                .withSupplyCurrentLimit(37.0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.50);
    }

    public interface Ports {
        int PIVOT_MOTOR = 20;
        int ROLLER_LEADER_MOTOR = 21;
        int ROLLER_FOLLOWER_MOTOR = 22;
    }
}
