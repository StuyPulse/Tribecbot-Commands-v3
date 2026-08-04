package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class Spindexer extends Mechanism {
    private static final Spindexer instance;

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Spindexer(new SpindexerIOTalonFX());

            case SIM -> instance = new Spindexer(new SpindexerIOSim());

            default -> instance = new Spindexer(new SpindexerIO() {
            });
        }
    }

    public static Spindexer getInstance() {
        return instance;
    }

    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs;
    private final SpindexerIOOutputs outputs;

    private Spindexer(SpindexerIO io) {
        this.io = io;
        this.inputs = new SpindexerIOInputsAutoLogged();
        this.outputs = new SpindexerIOOutputs();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    public void periodicAfterScheduler() {
        Logger.recordOutput("Spindexer/Duty Cycle Setpoint", outputs.spindexerLeaderDutyCycle);
        io.applyOutputs(outputs);
    }

    private void runDutyCycle(double dutyCycle) {
        outputs.spindexerLeaderDutyCycle = dutyCycle;
    }

    public Command runSpindexerForward() {
        return run(coroutine -> runDutyCycle(Settings.Spindexer.FORWARD_DUTY_CYCLE)).named("Spindexer Forward");
    }

    public Command runSpindexerReverse() {
        return run(coroutine -> runDutyCycle(Settings.Spindexer.REVERSE_DUTY_CYCLE)).named("Spindexer Reverse");
    }

    public Command stopSpindexer() {
        return run(coroutine -> runDutyCycle(0)).named("Spindexer Stop");
    }
}
