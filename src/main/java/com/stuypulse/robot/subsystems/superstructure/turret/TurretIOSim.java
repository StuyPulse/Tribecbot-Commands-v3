package com.stuypulse.robot.subsystems.superstructure.turret;

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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class TurretIOSim implements TurretIO {

    private SystemSim<FlywheelSim> sim;
    private PositionVoltage controller;
    private TalonFXSimulation simMotor;

    private StatusSignal<Current> turretSimMotorSupplyCurrent;
    private StatusSignal<Current> turretSimMotorStatorCurrent;
    private StatusSignal<Temperature> turretSimMotorTemperature;
    private StatusSignal<Angle> turretSimMotorPosition;
    private StatusSignal<Voltage> turretSimMotorAppliedVoltage;
    private StatusSignal<AngularVelocity> turretSimMotorVelocity;

    public TurretIOSim(){
        sim = SystemSim.of(
            new FlywheelSim(
                Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(1), 
                0, 
                Settings.Superstructure.Turret.GEAR_RATIO_MOTOR_TO_MECH), 
                DCMotor.getKrakenX60(1), 
            2.8)
        );

        simMotor =
        new TalonFXSimulation(
            Ports.Superstructure.Turret.MOTOR,
            Settings.Superstructure.Turret.GEAR_RATIO_MOTOR_TO_MECH,
            sim);

        controller = new PositionVoltage(0).withEnableFOC(true);

        turretSimMotorPosition = simMotor.getPosition();
        turretSimMotorSupplyCurrent = simMotor.getSupplyCurrent();
        turretSimMotorStatorCurrent = simMotor.getStatorCurrent();
        turretSimMotorTemperature = simMotor.getDeviceTemp();
        turretSimMotorAppliedVoltage = simMotor.getMotorVoltage();
        turretSimMotorVelocity = simMotor.getVelocity();
    }

    @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretSimMotorPosition,
        turretSimMotorSupplyCurrent,
        turretSimMotorStatorCurrent,
        turretSimMotorTemperature,
        turretSimMotorAppliedVoltage,
        turretSimMotorVelocity);
    inputs.turretMotorPosition = turretSimMotorPosition.getValue();
    inputs.turretMotorSupplyCurrent = turretSimMotorSupplyCurrent.getValue();
    inputs.turretMotorStatorCurrent = turretSimMotorStatorCurrent.getValue();
    inputs.turretMotorTemperature = turretSimMotorTemperature.getValue();
    inputs.turretMotorAppliedVoltage = turretSimMotorAppliedVoltage.getValue();
    inputs.turretMotorVelocity = turretSimMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    simMotor.setControl(controller.withPosition(outputs.turretPosition));
  }
}
