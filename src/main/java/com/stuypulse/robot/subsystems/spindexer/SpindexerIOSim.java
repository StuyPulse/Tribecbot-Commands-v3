package com.stuypulse.robot.subsystems.spindexer;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class SpindexerIOSim extends SpindexerIOBase {
    private final SystemSim<FlywheelSim> spindexerSim;

    private final TalonFXSimulation spindexerLeaderMotor;
    private final TalonFXSimulation spindexerFollowerMotor;

    public SpindexerIOSim() {
        final double gearing = SpindexerConstants.Settings.GEAR_RATIO;
        final SystemSim<FlywheelSim> spindexerSim = SystemSim.of(
                new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(
                                DCMotor.getKrakenX60(1), 0.01, gearing),
                        DCMotor.getKrakenX60(1),
                        0.01));

        final TalonFXSimulation spindexerLeaderMotor = new TalonFXSimulation(
                SpindexerConstants.Ports.LEADER_MOTOR,
                gearing,
                spindexerSim);
        final TalonFXSimulation spindexerFollowerMotor = new TalonFXSimulation(
                SpindexerConstants.Ports.FOLLOWER_MOTOR,
                gearing,
                spindexerSim);

        super(spindexerLeaderMotor, spindexerFollowerMotor);

        this.spindexerSim = spindexerSim;
        this.spindexerLeaderMotor = spindexerLeaderMotor;
        this.spindexerFollowerMotor = spindexerFollowerMotor;
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        spindexerSim.update(Settings.DT);

        spindexerLeaderMotor.refresh();
        spindexerFollowerMotor.refresh();

        super.updateInputs(inputs);
    }
}