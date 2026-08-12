package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;

import org.littletonrobotics.junction.Logger;

public class LEDController extends FullSubsystem {
    private static final LEDController instance; // LED instance

    static {
        switch (Settings.currentMode) {
            case REAL, SIM -> instance = new LEDController(new LEDIOCANdle() {});

            default -> instance = new LEDController(new LEDIO() {});
        }
    }

    public static LEDController getInstance() { // getter
        return instance;
    }

    // IO fields
    private final LEDIO io;
    private final LEDIOInputsAutoLogged inputs;
    private final LEDIOOutputs outputs;

    // CANdle

    public LEDController(LEDIO io) { // might have to be private
        this.io = io;
        this.inputs = new LEDIOInputsAutoLogged();
        this.outputs = new LEDIOOutputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
