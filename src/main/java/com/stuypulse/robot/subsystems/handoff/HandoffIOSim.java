package com.stuypulse.robot.subsystems.handoff;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class HandoffIOSim extends HandoffIOBase {
    private final SystemSim<FlywheelSim> handoffSim;

    private final TalonFXSimulation handoffLeaderMotor;
    private final TalonFXSimulation handoffFollowerMotor;

    public HandoffIOSim() {
        final SystemSim<FlywheelSim> handoffSim = SystemSim.of(
                new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(
                                DCMotor.getKrakenX60(1), 0.0001, 1),
                        DCMotor.getKrakenX60(1),
                        0.01));

        final TalonFXSimulation handoffLeaderMotor = new TalonFXSimulation(
                Ports.Handoff.LEADER_MOTOR,
                HandoffConstants.Handoff.GEAR_RATIO,
                handoffSim);
        final TalonFXSimulation handoffFollowerMotor = new TalonFXSimulation(
                Ports.Handoff.FOLLOWER_MOTOR,
                HandoffConstants.Handoff.GEAR_RATIO,
                handoffSim);

        super(handoffLeaderMotor, handoffFollowerMotor);

        this.handoffSim = handoffSim;
        this.handoffLeaderMotor = handoffLeaderMotor;
        this.handoffFollowerMotor = handoffFollowerMotor;
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        handoffSim.update(Settings.DT);

        handoffLeaderMotor.refresh();
        handoffFollowerMotor.refresh();

        super.updateInputs(inputs);
    }    
}
