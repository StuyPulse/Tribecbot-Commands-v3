/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.signals.*;

import com.stuypulse.robot.util.talonfx.TalonFXConfig;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {
    /** Classes to store all of the values a motor needs */

    public interface Superstructure {
        public interface Shooter {
            TalonFXConfig SHOOTER_MOTOR = new TalonFXConfig()
                    .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withSupplyCurrentLimitEnabled(false)
                    .withStatorCurrentLimitEnabled(false)
                    .withPIDConstants(
                            Gains.Superstructure.Shooter.kP.get(),
                            Gains.Superstructure.Shooter.kI.get(),
                            Gains.Superstructure.Shooter.kD.get(),
                            0)
                    .withFFConstants(
                            Gains.Superstructure.Shooter.kS.get(),
                            Gains.Superstructure.Shooter.kV.get(),
                            Gains.Superstructure.Shooter.kA.get(),
                            0)
                    .withSensorToMechanismRatio(Settings.Superstructure.Shooter.GEAR_RATIO)
                    .withStatorCurrentLimit(140)
                    .withStatorCurrentLimitEnabled(false)
                    .withSupplyCurrentLimit(100)
                    .withSupplyCurrentLimitEnabled(true)
                    .withLowerLimitSupplyCurrent(60, 1);
        }

        public interface Hood {
            TalonFXConfig HOOD_MOTOR = new TalonFXConfig()
                    .withInvertedValue(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withSupplyCurrentLimit(80.0)
                    .withStatorCurrentLimitEnabled(false)
                    .withRampRate(0.25)
                    .withPIDConstants(
                            Gains.Superstructure.Hood.kP,
                            Gains.Superstructure.Hood.kI,
                            Gains.Superstructure.Hood.kD,
                            0)
                    .withFFConstants(
                            Gains.Superstructure.Hood.kS,
                            Gains.Superstructure.Hood.kV,
                            Gains.Superstructure.Hood.kA,
                            0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
                    .withSensorToMechanismRatio(Settings.Superstructure.Hood.GEAR_RATIO)
                    .withSoftLimits(
                            true,
                            true,
                            Settings.Superstructure.Hood.FORWARD_SOFT_LIMIT.in(Rotations),
                            Settings.Superstructure.Hood.REVERSE_SOFT_LIMIT.in(Rotations));
        }
    }
}
