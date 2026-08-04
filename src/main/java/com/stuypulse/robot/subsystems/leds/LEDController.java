package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.leds.LEDIO.LEDIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Mechanism;

public class LEDController extends Mechanism {
    private static final LEDController instance; // LED instance

    static {
        if (Robot.isReal() || Robot.isSimulation()) {
            instance = new LEDController(new LEDIOCANdle() {
            });
        } else {
            instance = new LEDController(new LEDIO() {
            });
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

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDs", inputs);
    }

    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
