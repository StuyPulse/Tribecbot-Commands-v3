/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

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

    public static class CANCoderConfig {
        private final CANcoderConfiguration configuration = new CANcoderConfiguration();
        private final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

        public void configure(CANcoder encoder) {
            CANcoderConfiguration defaultConfig = new CANcoderConfiguration();
            encoder.getConfigurator().apply(defaultConfig);

            encoder.getConfigurator().apply(configuration);
        }

        public CANcoderConfiguration getConfiguration() {
            return this.configuration;
        }

        // MAGNET SENSOR CONFIGS

        public CANCoderConfig withSensorDirection(SensorDirectionValue sensorDirection) {
            magnetSensorConfigs.SensorDirection = sensorDirection;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }

        public CANCoderConfig withAbsoluteSensorDiscontinuityPoint(double discontinuityPoint) {
            magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }

        public CANCoderConfig withMagnetOffset(double magnetOffset) {
            magnetSensorConfigs.MagnetOffset = magnetOffset;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }
    }

    /** Wrapper class for configuring TalonFX motors */
    public static class TalonFXConfig {
        private final TalonFXConfiguration configuration = new TalonFXConfiguration();
        private final Slot0Configs slot0Configs = new Slot0Configs();
        private final Slot1Configs slot1Configs = new Slot1Configs();
        private final Slot2Configs slot2Configs = new Slot2Configs();
        private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        private final OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        private final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        private final ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        private final VoltageConfigs voltageConfigs = new VoltageConfigs();
        private final TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs();

        private final double[] lastKP = new double[3];
        private final double[] lastKI = new double[3];
        private final double[] lastKD = new double[3];
        private final double[] lastKS = new double[3];
        private final double[] lastKV = new double[3];
        private final double[] lastKA = new double[3];

        public void configure(TalonFX motor) {
            TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
            motor.getConfigurator().apply(defaultConfig);

            motor.getConfigurator().apply(configuration);
        }

        public TalonFXConfiguration getConfiguration() {
            return this.configuration;
        }

        public void updateGainsConfig(
                TalonFX motor,
                int slot,
                double kP,
                double kI,
                double kD,
                double kS,
                double kV,
                double kA) {
            if (slot != 0 && slot != 1 && slot != 2) {
                return;
            }

            final boolean changed = kP != lastKP[slot]
                    || kI != lastKI[slot]
                    || kD != lastKD[slot]
                    || kS != lastKS[slot]
                    || kV != lastKV[slot]
                    || kA != lastKA[slot];

            if (!changed) {
                return;
            }

            final SlotConfigs gainConfig = new SlotConfigs()
                    .withKP(kP)
                    .withKI(kI)
                    .withKD(kD)
                    .withKS(kS)
                    .withKV(kV)
                    .withKA(kA);

            gainConfig.SlotNumber = slot;

            motor.getConfigurator().apply(gainConfig);

            lastKP[slot] = kP;
            lastKI[slot] = kI;
            lastKD[slot] = kD;
            lastKS[slot] = kS;
            lastKV[slot] = kV;
            lastKA[slot] = kA;

            switch (slot) {
                case 0:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot0);
                    break;
                case 1:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot1);
                    break;
                case 2:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot2);
                    break;
            }
        }

        // SLOT CONFIGS

        public TalonFXConfig withPIDConstants(double kP, double kI, double kD, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.withKP(kP).withKI(kI).withKD(kD);
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.withKP(kP).withKI(kI).withKD(kD);
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.withKP(kP).withKI(kI).withKD(kD);
                    configuration.withSlot2(slot2Configs);
                    break;
            }
            return this;
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA, int slot) {
            return withFFConstants(kS, kV, kA, 0.0, slot);
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA, double kG, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                    configuration.withSlot2(slot2Configs);
                    break;
            }
            return this;
        }

        public TalonFXConfig withStaticFeedforwardSign(
                StaticFeedforwardSignValue staticFeedforwardSign, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.withStaticFeedforwardSign(staticFeedforwardSign);
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.withStaticFeedforwardSign(staticFeedforwardSign);
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.withStaticFeedforwardSign(staticFeedforwardSign);
                    configuration.withSlot2(slot2Configs);
                    break;
            }

            return this;
        }

        public TalonFXConfig withGravityType(GravityTypeValue gravityType) {
            slot0Configs.withGravityType(gravityType);
            slot1Configs.withGravityType(gravityType);
            slot2Configs.withGravityType(gravityType);

            configuration.withSlot0(slot0Configs);
            configuration.withSlot1(slot1Configs);
            configuration.withSlot2(slot2Configs);

            return this;
        }

        /**
         * Modifies this configuration's GainSchedErrorThreshold parameter, which is the
         * position
         * closed-loop error threshold to apply gains scheduling.
         *
         * @param value              Gains scheduling behavior, see {@link
         *                           com.ctre.phoenix6.signals.GainSchedBehaviorValue}.
         * @param thresholdRotations The position closed-loop error threshold for gain
         *                           scheduling in
         *                           rotations.
         * @param slot               Slot to apply gains scheduling behavior to.
         * @return Itself for method-chaining.
         */
        public TalonFXConfig withGainSchedBehavior(
                GainSchedBehaviorValue value, double thresholdRotations, int slot) {
            return this.withGainSchedBehavior(value, Rotations.of(thresholdRotations), slot);
        }

        /**
         * Modifies this configuration's GainSchedErrorThreshold parameter, which is the
         * position
         * closed-loop error threshold to apply gains scheduling.
         *
         * @param value              Gains scheduling behavior, see {@link
         *                           com.ctre.phoenix6.signals.GainSchedBehaviorValue}.
         * @param thresholdRotations The position closed-loop error threshold for gain
         *                           scheduling.
         * @param slot               Slot to apply gains scheduling behavior to.
         * @return Itself for method-chaining.
         */
        public TalonFXConfig withGainSchedBehavior(
                GainSchedBehaviorValue value, Angle threshold, int slot) {
            closedLoopGeneralConfigs.withGainSchedErrorThreshold(threshold);
            configuration.withClosedLoopGeneral(closedLoopGeneralConfigs);

            switch (slot) {
                case 0: {
                    slot0Configs.withGainSchedBehavior(value);
                    configuration.withSlot0(slot0Configs);
                }
                    break;
                case 1: {
                    slot1Configs.withGainSchedBehavior(value);
                    configuration.withSlot1(slot1Configs);
                }
                    break;
                case 2: {
                    slot2Configs.withGainSchedBehavior(value);
                    configuration.withSlot2(slot2Configs);
                }
                    break;
            }

            return this;
        }

        // MOTOR OUTPUT CONFIGS

        public TalonFXConfig withInvertedValue(InvertedValue invertedValue) {
            motorOutputConfigs.withInverted(invertedValue);

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        public TalonFXConfig withNeutralMode(NeutralModeValue neutralMode) {
            motorOutputConfigs.withNeutralMode(neutralMode);

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        /**
         * Modifies this configuration's velocity filter's time constant.
         *
         * @param filterTime The configurable time constant in seconds of the Kalman
         *                   velocity
         *                   filter.
         * @return Itself for easier method-chaining
         */
        public TalonFXConfig withVelocityTimeFilter(double filterTimeSeconds) {
            return this.withVelocityTimeFilter(Seconds.of(filterTimeSeconds));
        }

        /**
         * Modifies this configuration's velocity filter's time constant.
         *
         * @param filterTime The configurable time constant of the Kalman velocity
         *                   filter.
         * @return Itself for easier method-chaining
         */
        public TalonFXConfig withVelocityTimeFilter(Time filterTime) {
            feedbackConfigs.withVelocityFilterTimeConstant(filterTime);

            configuration.withFeedback(feedbackConfigs);

            return this;
        }

        // RAMP RATE CONFIGS

        /**
         * Modifies this configuration's open and closed loop ramp periods for the
         * following control
         * types:
         *
         * <ul>
         * <li>DutyCycle
         * <li>TorqueCurrent
         * <li>Voltage
         * </ul>
         *
         * <br>
         *
         * @param rampRateSeconds Seconds for the motor to reach peak output from 0
         * @return Itself for method-chaining
         */
        public TalonFXConfig withRampRate(double rampRateSeconds) {
            return this.withRampRate(Seconds.of(rampRateSeconds));
        }

        /**
         * Modifies this configuration's open and closed loop ramp periods for the
         * following control
         * types:
         *
         * <ul>
         * <li>DutyCycle
         * <li>TorqueCurrent
         * <li>Voltage
         * </ul>
         *
         * <br>
         *
         * @param rampRateSeconds Time for the motor to reach peak output from 0
         * @return Itself for easy method chaining
         */
        public TalonFXConfig withRampRate(Time rampRate) {
            closedLoopRampsConfigs.withDutyCycleClosedLoopRampPeriod(rampRate);
            closedLoopRampsConfigs.withTorqueClosedLoopRampPeriod(rampRate);
            closedLoopRampsConfigs.withVoltageClosedLoopRampPeriod(rampRate);

            openLoopRampsConfigs.withDutyCycleOpenLoopRampPeriod(rampRate);
            openLoopRampsConfigs.withTorqueOpenLoopRampPeriod(rampRate);
            openLoopRampsConfigs.withVoltageOpenLoopRampPeriod(rampRate);

            configuration.withClosedLoopRamps(closedLoopRampsConfigs);
            configuration.withOpenLoopRamps(openLoopRampsConfigs);

            return this;
        }

        // CURRENT LIMIT CONFIGS
        /**
         * Modifies this configuration's supply current lower limit.
         *
         * @param currentLowerLimit The amount of supply current in amperes allowed
         *                          after the
         *                          regular
         *                          {@link CurrentLimitsConfigs#SupplyCurrentLimit} is
         *                          active for longer than the
         *                          time parameter.
         * @param time              The amount of time in seconds that the regular
         *                          {@link
         *                          CurrentLimitsConfigs#SupplyCurrentLimit} can be
         *                          active before lowring it to the
         *                          currentLowerLimit parameter.
         * @return
         */
        public TalonFXConfig withLowerLimitSupplyCurrent(
                double currentLowerLimitAmps, double timeSeconds) {
            return this.withLowerLimitSupplyCurrent(
                    Amps.of(currentLowerLimitAmps), Seconds.of(timeSeconds));
        }

        /**
         * Modifies this configuration's supply current lower limit.
         *
         * @param currentLowerLimit The amount of supply current allowed after the
         *                          regular {@link
         *                          CurrentLimitsConfigs#SupplyCurrentLimit} is active
         *                          for longer than the time
         *                          parameter.
         * @param time              The amount of time that the regular {@link
         *                          CurrentLimitsConfigs#SupplyCurrentLimit} can be
         *                          active before lowring it to the
         *                          currentLowerLimit parameter.
         * @return
         */
        public TalonFXConfig withLowerLimitSupplyCurrent(Current currentLowerLimit, Time time) {
            currentLimitsConfigs.withSupplyCurrentLowerLimit(currentLowerLimit);
            currentLimitsConfigs.withSupplyCurrentLowerTime(time);

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        /**
         * Modifies this configuration's supply current limit and enables it.
         *
         * @param currentLimit Maximum allowed current in amperes drawn from the
         *                     battery.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withSupplyCurrentLimit(double currentLimitAmps) {
            return this.withSupplyCurrentLimit(Amps.of(currentLimitAmps));
        }

        /**
         * Modifies this configuration's supply current limit and enables it.
         *
         * @param currentLimit Maximum allowed current drawn from the battery.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withSupplyCurrentLimit(Current currentLimit) {
            currentLimitsConfigs.withSupplyCurrentLimit(currentLimit);
            currentLimitsConfigs.withSupplyCurrentLimitEnable(true);

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        public TalonFXConfig withSupplyCurrentLimitEnabled(boolean enabled) {
            currentLimitsConfigs.withSupplyCurrentLimitEnable(enabled);

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        /**
         * Modifies this configuration's stator current limit and enables it.
         *
         * @param currentLimit Amount of current in amperes allowed in the motor
         *                     (motoring and regen
         *                     current).
         * @return Itself for easier method chaining.
         */
        public TalonFXConfig withStatorCurrentLimit(double currentLimitAmps) {
            return this.withStatorCurrentLimit(Amps.of(currentLimitAmps));
        }

        /**
         * Modifies this configuration's stator current limit and enables it.
         *
         * @param currentLimit Amount of current allowed in the motor (motoring and
         *                     regen current).
         * @return Itself for easier method chaining.
         */
        public TalonFXConfig withStatorCurrentLimit(Current currentLimit) {
            currentLimitsConfigs.withStatorCurrentLimit(currentLimit);
            currentLimitsConfigs.withStatorCurrentLimitEnable(true);

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        public TalonFXConfig withStatorCurrentLimitEnabled(boolean enabled) {
            currentLimitsConfigs.withStatorCurrentLimitEnable(enabled);

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        /**
         * Modifies this configuration's torque current limits. Limits apply during
         * torque current
         * control modes.
         *
         * @param peakForwardTorqueCurrent Maximum forward torque current output in
         *                                 amperes.
         * @param peakReverseTorqueCurrent Maximum reverse torque current output in
         *                                 amperes.
         * @param neutralTolerance         Current range in amperes where requested
         *                                 torque current will
         *                                 result in zero bridge output.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withTorqueCurrentLimits(
                double peakForwardTorqueCurrentAmps,
                double peakReverseTorqueCurrentAmps,
                double neutralToleranceAmps) {
            return this.withTorqueCurrentLimits(
                    Amps.of(peakForwardTorqueCurrentAmps),
                    Amps.of(peakReverseTorqueCurrentAmps),
                    Amps.of(neutralToleranceAmps));
        }

        /**
         * Modifies this configuration's torque current limits. Limits apply during
         * torque current
         * control modes.
         *
         * @param peakForwardTorqueCurrent Maximum forward torque current output.
         * @param peakReverseTorqueCurrent Maximum reverse torque current output.
         * @param neutralTolerance         Current range where requested torque current
         *                                 will result in zero
         *                                 bridge output.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withTorqueCurrentLimits(
                Current peakForwardTorqueCurrent,
                Current peakReverseTorqueCurrent,
                Current neutralTolerance) {
            torqueCurrentConfigs.withPeakForwardTorqueCurrent(peakForwardTorqueCurrent);
            torqueCurrentConfigs.withPeakReverseTorqueCurrent(peakReverseTorqueCurrent);
            torqueCurrentConfigs.withTorqueNeutralDeadband(neutralTolerance);

            configuration.withTorqueCurrent(torqueCurrentConfigs);

            return this;
        }

        // VOLTAGE LIMIT CONFIGS

        /**
         * Modifies this configuration's voltage limits. Limits apply during voltage
         * control modes.
         *
         * @param peakForwardVoltage Maximum forward voltage output in volts.
         * @param peakReverseVoltage Maximum reverse voltage output in volts.
         * @return Itself for easier method chaining.
         */
        public TalonFXConfig withVoltageLimits(double peakForwardVolts, double peakReverseVolts) {
            return this.withVoltageLimits(Volts.of(peakForwardVolts), Volts.of(peakReverseVolts));
        }

        /**
         * Modifies this configuration's voltage limits. Limits apply during voltage
         * control modes.
         *
         * @param peakForwardVoltage Maximum forward voltage output.
         * @param peakReverseVoltage Maximum reverse voltage output.
         * @return Itself for easier method chaining.
         */
        public TalonFXConfig withVoltageLimits(
                Voltage peakForwardVoltage, Voltage peakReverseVoltage) {
            voltageConfigs.withPeakForwardVoltage(peakForwardVoltage);
            voltageConfigs.withPeakReverseVoltage(peakReverseVoltage);

            configuration.withVoltage(voltageConfigs);

            return this;
        }

        // SOFTWARE LIMIT CONFIGS
        /**
         * Modifies this configuration's software limit switch.
         *
         * @param forwardEnable    Whether to set the motor output to neutral if its
         *                         position exceeds
         *                         the forward threshold.
         * @param reverseEnable    Whether to set the motor output to neutral if its
         *                         position exceeds
         *                         the reverse threshold.
         * @param forwardThreshold The threshold angle in rotations for the application
         *                         of the
         *                         forward limit.
         * @param reverseThreshold The threshold angle in rotations for the application
         *                         of the
         *                         reverse limit.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withSoftLimits(
                boolean forwardEnable,
                boolean reverseEnable,
                double forwardThresholdRotations,
                double reverseThresholdRotations) {
            return this.withSoftLimits(
                    forwardEnable,
                    reverseEnable,
                    Rotations.of(forwardThresholdRotations),
                    Rotations.of(reverseThresholdRotations));
        }

        /**
         * Modifies this configuration's software limit switch.
         *
         * @param forwardEnable    Whether to set the motor output to neutral if its
         *                         position exceeds
         *                         the forward threshold.
         * @param reverseEnable    Whether to set the motor output to neutral if its
         *                         position exceeds
         *                         the reverse threshold.
         * @param forwardThreshold The threshold angle for the application of the
         *                         forward limit.
         * @param reverseThreshold The threshold angle for the application of the
         *                         reverse limit.
         * @return Itself for easier method-chaining.
         */
        public TalonFXConfig withSoftLimits(
                boolean forwardEnable,
                boolean reverseEnable,
                Angle forwardThreshold,
                Angle reverseThreshold) {
            softwareLimitSwitchConfigs.withForwardSoftLimitEnable(forwardEnable);
            softwareLimitSwitchConfigs.withReverseSoftLimitEnable(reverseEnable);
            softwareLimitSwitchConfigs.withForwardSoftLimitThreshold(forwardThreshold);
            softwareLimitSwitchConfigs.withReverseSoftLimitThreshold(reverseThreshold);

            configuration.withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

            return this;
        }

        // MOTION MAGIC CONFIGS
        /**
         * Modifies this configuration's motion magic profile.
         *
         * @param maxVelocity     Maximum/cruise velocity of the motion profile in
         *                        rotations per second.
         * @param maxAcceleration Maximum acceleration of the motion profile in
         *                        rotations per second
         *                        squared.
         * @return Itself for method-chaining.
         */
        public TalonFXConfig withMotionProfile(
                double maxVelocityRotPerSec, double maxAccelerationRotPerSecSquared) {
            return this.withMotionProfile(
                    RotationsPerSecond.of(maxVelocityRotPerSec),
                    RotationsPerSecondPerSecond.of(maxAccelerationRotPerSecSquared));
        }

        /**
         * Modifies this configuration's motion magic profile.
         *
         * @param maxVelocity     Maximum/cruise velocity of the motion profile.
         * @param maxAcceleration Maximum acceleration of the motion profile.
         * @return Itself for method-chaining.
         */
        public TalonFXConfig withMotionProfile(
                AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
            motionMagicConfigs.withMotionMagicCruiseVelocity(maxVelocity);
            motionMagicConfigs.withMotionMagicAcceleration(maxAcceleration);

            configuration.withMotionMagic(motionMagicConfigs);

            return this;
        }

        // FEEDBACK CONFIGS

        public TalonFXConfig withRemoteSensor(
                int ID, FeedbackSensorSourceValue source, double rotorToSensorRatio) {
            feedbackConfigs.withFeedbackRemoteSensorID(ID);
            feedbackConfigs.withFeedbackSensorSource(source);
            feedbackConfigs.withRotorToSensorRatio(rotorToSensorRatio);

            configuration.withFeedback(feedbackConfigs);

            return this;
        }

        public TalonFXConfig withSensorToMechanismRatio(double sensorToMechanismRatio) {
            feedbackConfigs.withSensorToMechanismRatio(sensorToMechanismRatio);

            configuration.withFeedback(feedbackConfigs);

            return this;
        }
    }
}
