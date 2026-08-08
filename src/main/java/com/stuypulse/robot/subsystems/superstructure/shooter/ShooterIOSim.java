package com.stuypulse.robot.subsystems.superstructure.shooter;

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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class ShooterIOSim implements ShooterIO {

    //Sims
    private final SystemSim<FlywheelSim> flywheelSim;

    private final TalonFXSimulation shooterLeaderSim;
    private final TalonFXSimulation shooterFollowerSim;

    //Leader Data
    private final StatusSignal<Temperature> shooterLeaderSimTemperature;
    private final StatusSignal<Current> shooterLeaderSimSupplyCurrent;
    private final StatusSignal<Current> shooterLeaderSimStatorCurrent;
    private final StatusSignal<Angle> shooterLeaderSimPosition;
    private final StatusSignal<AngularVelocity> shooterLeaderSimVelocity;
    private final StatusSignal<Voltage> shooterLeaderSimVoltage;
    
    //Follower Data
    private final StatusSignal<Temperature> shooterFollowerSimTemperature;
    private final StatusSignal<Current> shooterFollowerSimSupplyCurrent;
    private final StatusSignal<Current> shooterFollowerSimStatorCurrent;
    private final StatusSignal<Angle> shooterFollowerSimPosition;
    private final StatusSignal<AngularVelocity> shooterFollowerSimVelocity;
    private final StatusSignal<Voltage> shooterFollowerSimVoltage;

    //Controllers
    private final VelocityTorqueCurrentFOC shooterLeaderController;
    private final Follower shooterFollowerController;

    public ShooterIOSim() {

        flywheelSim = 
            SystemSim.of(
                new FlywheelSim(
                    Models.flywheelFromPhysicalConstants(
                        DCMotor.getKrakenX44(2), 0.05, Settings.Superstructure.Shooter.GEAR_RATIO), 
                        DCMotor.getKrakenX44(2), Settings.Superstructure.Shooter.GEAR_RATIO)
        );

        shooterLeaderSim = 
            new TalonFXSimulation(
                Ports.Superstructure.Shooter.MOTOR_LEAD,
                Settings.Superstructure.Shooter.GEAR_RATIO, 
                flywheelSim);
        shooterFollowerSim = 
            new TalonFXSimulation(
                Ports.Superstructure.Shooter.MOTOR_FOLLOW, 
                Settings.Superstructure.Shooter.GEAR_RATIO, 
                flywheelSim);

        shooterLeaderSim.configure(Motors.Superstructure.Shooter.SHOOTER_MOTOR);
        shooterFollowerSim.configure(Motors.Superstructure.Shooter.SHOOTER_MOTOR);

        shooterLeaderController = new VelocityTorqueCurrentFOC(0);

        shooterFollowerController = 
            new Follower(shooterLeaderSim.getDeviceID(), MotorAlignmentValue.Opposed);
        shooterFollowerSim.setControl(shooterFollowerController);
        

        shooterLeaderSimPosition = shooterLeaderSim.getPosition();
        shooterLeaderSimSupplyCurrent = shooterLeaderSim.getSupplyCurrent();
        shooterLeaderSimStatorCurrent = shooterLeaderSim.getStatorCurrent();
        shooterLeaderSimTemperature = shooterLeaderSim.getDeviceTemp(); 
        shooterLeaderSimVoltage = shooterLeaderSim.getMotorVoltage();
        shooterLeaderSimVelocity = shooterLeaderSim.getVelocity();

        shooterFollowerSimPosition = shooterFollowerSim.getPosition();
        shooterFollowerSimSupplyCurrent = shooterFollowerSim.getSupplyCurrent();
        shooterFollowerSimStatorCurrent = shooterFollowerSim.getStatorCurrent();
        shooterFollowerSimTemperature = shooterFollowerSim.getDeviceTemp();
        shooterFollowerSimVoltage = shooterFollowerSim.getMotorVoltage();
        shooterFollowerSimVelocity = shooterFollowerSim.getVelocity();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterLeaderSim.refresh();
    shooterFollowerSim.refresh();

    BaseStatusSignal.refreshAll(
        shooterLeaderSimPosition,
        shooterLeaderSimSupplyCurrent,
        shooterLeaderSimStatorCurrent,
        shooterLeaderSimTemperature,
        shooterLeaderSimVoltage,
        shooterLeaderSimVelocity,
        shooterFollowerSimPosition,
        shooterFollowerSimSupplyCurrent,
        shooterFollowerSimStatorCurrent,
        shooterFollowerSimTemperature,
        shooterFollowerSimVoltage,
        shooterFollowerSimVelocity);

    inputs.shooterLeaderMotorPosition = shooterLeaderSimPosition.getValue();
    inputs.shooterLeaderMotorSupplyCurrent = shooterLeaderSimSupplyCurrent.getValue();
    inputs.shooterLeaderMotorStatorCurrent = shooterLeaderSimStatorCurrent.getValue();
    inputs.shooterLeaderMotorTemperature = shooterLeaderSimTemperature.getValue();
    inputs.shooterLeaderMotorAppliedVoltage = shooterLeaderSimVoltage.getValue();
    inputs.shooterLeaderMotorVelocity = shooterLeaderSimVelocity.getValue();

    inputs.shooterFollowerMotorPosition = shooterFollowerSimPosition.getValue();
    inputs.shooterFollowerMotorSupplyCurrent = shooterFollowerSimSupplyCurrent.getValue();
    inputs.shooterFollowerMotorStatorCurrent = shooterFollowerSimStatorCurrent.getValue();
    inputs.shooterFollowerMotorTemperature = shooterFollowerSimTemperature.getValue();
    inputs.shooterFollowerMotorAppliedVoltage = shooterFollowerSimVoltage.getValue();
    inputs.shooterFollowerMotorVelocity = shooterFollowerSimVelocity.getValue();
    }

    @Override
    public void applyOutputs(ShooterIOOutputs outputs) {
        shooterLeaderSim.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
    }
}