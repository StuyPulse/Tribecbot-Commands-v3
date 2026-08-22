package com.stuypulse.robot.subsystems.superstructure.shooter;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class ShooterIOSim extends ShooterIOBase {

    //Sims
    private final SystemSim<FlywheelSim> flywheelSim;

    private final TalonFXSimulation shooterLeaderSim;
    private final TalonFXSimulation shooterFollowerSim;

    //Controllers
    private final VelocityTorqueCurrentFOC shooterLeaderController;
    private final Follower shooterFollowerController;

    public ShooterIOSim() {

        final SystemSim<FlywheelSim> flywheelSim = 
            SystemSim.of(
                new FlywheelSim(
                    Models.flywheelFromPhysicalConstants(
                        DCMotor.getKrakenX44(2), 0.05, SuperstructureConstants.Shooter.Settings.GEAR_RATIO), 
                        DCMotor.getKrakenX44(2), SuperstructureConstants.Shooter.Settings.GEAR_RATIO)
        );

        final TalonFXSimulation shooterLeaderSim = 
            new TalonFXSimulation(
                Ports.Superstructure.Shooter.MOTOR_LEAD,
                SuperstructureConstants.Shooter.Settings.GEAR_RATIO, 
                flywheelSim);
        final TalonFXSimulation shooterFollowerSim = 
            new TalonFXSimulation(
                Ports.Superstructure.Shooter.MOTOR_FOLLOW, 
                SuperstructureConstants.Shooter.Settings.GEAR_RATIO, 
                flywheelSim);

        final VelocityTorqueCurrentFOC shooterLeaderController = new VelocityTorqueCurrentFOC(0);

        final Follower shooterFollowerController = 
            new Follower(shooterLeaderSim.getDeviceID(), MotorAlignmentValue.Opposed);
        shooterFollowerSim.setControl(shooterFollowerController);
        
        super(shooterLeaderSim, shooterFollowerSim);

        this.shooterFollowerSim = shooterFollowerSim;
        this.shooterLeaderSim = shooterLeaderSim;
        this.flywheelSim = flywheelSim;
        this.shooterLeaderController = shooterLeaderController;
        this.shooterFollowerController = shooterFollowerController;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterLeaderSim.refresh();
        shooterFollowerSim.refresh();

        super.updateInputs(inputs);
    }

    @Override
    public void applyOutputs(ShooterIOOutputs outputs) {
        shooterLeaderSim.setControl(shooterLeaderController.withVelocity(outputs.shooterVelocity));
    }
}