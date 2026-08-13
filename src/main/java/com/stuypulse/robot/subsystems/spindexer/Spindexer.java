package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;

public class Spindexer extends FullSubsystem {
    private static final Spindexer instance;

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Spindexer(new SpindexerIOTalonFX());

            case SIM -> instance = new Spindexer(new SpindexerIOSim());

            default -> instance = new Spindexer(new SpindexerIO() {});
        }
    }

    public static Spindexer getInstance() {
        return instance;
    }

    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs;
    private final SpindexerIOOutputs outputs;

    @AutoLogOutput(key = "States/Spindexer")
    private SpindexerState state;

    private Spindexer(SpindexerIO io) {
        this.io = io;
        this.inputs = new SpindexerIOInputsAutoLogged();
        this.outputs = new SpindexerIOOutputs();

        setState(SpindexerState.STOP);
    }

    public enum SpindexerState {
        FORWARD,
        REVERSE,
        STOP
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        if (!Settings.EnabledSubsystems.SPINDEXER.get()) {
            stop();

            return;
        }

        switch (state) {
            case FORWARD -> runDutyCycle(SpindexerConstants.Settings.FORWARD_DUTY_CYCLE);
            case REVERSE -> runDutyCycle(SpindexerConstants.Settings.REVERSE_DUTY_CYCLE);
            case STOP -> stop();
        }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runDutyCycle(double dutyCycle) {
        outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.DUTY_CYCLE;
        outputs.spindexerLeaderDutyCycle = dutyCycle;
    }

    private void stop() {
        outputs.spindexerMode = SpindexerIO.SpindexerIOOutputMode.STOP;
    }

    private void setState(SpindexerState state) {
        this.state = state;
    }

    public Command runSpindexerForward() {
        return run(coroutine -> setState(SpindexerState.FORWARD)).named("Spindexer Forward");
    }

    public Command runSpindexerReverse() {
        return run(coroutine -> setState(SpindexerState.REVERSE)).named("Spindexer Reverse");
    }

    public Command stopSpindexer() {
        return run(coroutine -> setState(SpindexerState.STOP)).named("Spindexer Stop");
    }
}
