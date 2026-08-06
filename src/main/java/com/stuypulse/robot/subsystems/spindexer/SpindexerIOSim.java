package com.stuypulse.robot.subsystems.spindexer;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class SpindexerIOSim implements SpindexerIO {

    private final SystemSim<FlywheelSim> flywheelSim;

    private final TalonFXSimulation spindexerLeaderSim;
    private final TalonFXSimulation spindexerFollowerSim;

    private final StatusSignal<Angle> spindexerLeaderPosition;
    private final StatusSignal<Current> spindexerLeaderSupplyCurrent;
    private final StatusSignal<Current> spindexerLeaderStatorCurrent;
    private final StatusSignal<Temperature> spindexerLeaderTemperature;
    private final StatusSignal<Voltage> spindexerLeaderAppliedVoltage;
    private final StatusSignal<AngularVelocity> spindexerLeaderVelocity;

    private final StatusSignal<Angle> spindexerFollowerPosition;
    private final StatusSignal<Current> spindexerFollowerSupplyCurrent;
    private final StatusSignal<Temperature> spindexerFollowerTemperature;
    private final StatusSignal<Current> spindexerFollowerStatorCurrent;
    private final StatusSignal<Voltage> spindexerFollowerAppliedVoltage;
    private final StatusSignal<AngularVelocity> spindexerFollowerVelocity;

    private final DutyCycleOut spindexerController;
    private final Follower followerController;

    public SpindexerIOSim() {

        flywheelSim = 
            SystemSim.of(
                new FlywheelSim(
                    Models.flywheelFromPhysicalConstants(
                        DCMotor.getKrakenX60(1), 0.01, 1),
                    DCMotor.getKrakenX60(1),
                    0.01));

        spindexerLeaderSim = 
            new TalonFXSimulation(
                Ports.Spindexer.LEADER,
                Settings.Spindexer.GEAR_RATIO,
                flywheelSim);
        spindexerFollowerSim = 
            new TalonFXSimulation(
                Ports.Spindexer.LEADER, 
                Settings.Spindexer.GEAR_RATIO,
                flywheelSim);
        
        spindexerController = new DutyCycleOut(0);
        followerController = new Follower(spindexerLeaderSim.getDeviceID(), MotorAlignmentValue.Aligned);

        spindexerFollowerSim.setControl(followerController);

        spindexerLeaderPosition = spindexerLeaderSim.getPosition();
        spindexerLeaderSupplyCurrent = spindexerLeaderSim.getSupplyCurrent();
        spindexerLeaderStatorCurrent = spindexerLeaderSim.getStatorCurrent();
        spindexerLeaderTemperature = spindexerLeaderSim.getDeviceTemp();
        spindexerLeaderAppliedVoltage = spindexerLeaderSim.getMotorVoltage();
        spindexerLeaderVelocity = spindexerLeaderSim.getVelocity();

        spindexerFollowerPosition = spindexerFollowerSim.getPosition();
        spindexerFollowerSupplyCurrent = spindexerFollowerSim.getSupplyCurrent();
        spindexerFollowerStatorCurrent = spindexerFollowerSim.getStatorCurrent();
        spindexerFollowerTemperature = spindexerFollowerSim.getDeviceTemp();
        spindexerFollowerAppliedVoltage = spindexerFollowerSim.getMotorVoltage();
        spindexerFollowerVelocity = spindexerFollowerSim.getVelocity();
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                spindexerLeaderPosition,
                spindexerLeaderSupplyCurrent,
                spindexerLeaderStatorCurrent,
                spindexerLeaderTemperature,
                spindexerLeaderAppliedVoltage,
                spindexerLeaderVelocity,
                spindexerFollowerPosition,
                spindexerFollowerSupplyCurrent,
                spindexerFollowerStatorCurrent,
                spindexerFollowerTemperature,
                spindexerFollowerAppliedVoltage,
                spindexerFollowerVelocity);

        inputs.spindexerLeaderMotorPosition = spindexerLeaderPosition.getValue();
        inputs.spindexerLeaderMotorSupplyCurrent = spindexerLeaderSupplyCurrent.getValue();
        inputs.spindexerLeaderMotorStatorCurrent = spindexerLeaderStatorCurrent.getValue();
        inputs.spindexerLeaderMotorTemperature = spindexerLeaderTemperature.getValue();
        inputs.spindexerLeaderMotorAppliedVoltage = spindexerLeaderAppliedVoltage.getValue();
        inputs.spindexerLeaderMotorVelocity = spindexerLeaderVelocity.getValue();

        inputs.spindexerFollowerMotorPosition = spindexerFollowerPosition.getValue();
        inputs.spindexerFollowerMotorSupplyCurrent = spindexerFollowerSupplyCurrent.getValue();
        inputs.spindexerFollowerMotorStatorCurrent = spindexerFollowerStatorCurrent.getValue();
        inputs.spindexerFollowerMotorTemperature = spindexerFollowerTemperature.getValue();
        inputs.spindexerFollowerMotorAppliedVoltage = spindexerFollowerAppliedVoltage.getValue();
        inputs.spindexerFollowerMotorVelocity = spindexerFollowerVelocity.getValue();
    }

    @Override
    public void applyOutputs(SpindexerIOOutputs outputs) {
        spindexerLeaderSim.setControl(
                spindexerController.withOutput(outputs.spindexerLeaderDutyCycle));
    }
}