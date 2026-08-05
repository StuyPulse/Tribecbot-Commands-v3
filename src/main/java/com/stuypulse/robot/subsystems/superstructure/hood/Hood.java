package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodIO.HoodIOOutputs;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.littletonrobotics.junction.Logger;

public class Hood extends Mechanism {
    private static final Hood instance;

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Hood(new HoodIOTalonFX());

            case SIM -> instance = new Hood(new HoodIOSim());

            default -> instance = new Hood(new HoodIO() {
            });
        }
    }

    public static Hood getInstance() {
        return instance;
    }

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs;
    private final HoodIOOutputs outputs;

    // Using SOTM or FOTM
    private boolean OTM;

    private final Debouncer hoodStallingDebouncer;

    private Hood(HoodIO io) {
        this.io = io;
        inputs = new HoodIOInputsAutoLogged();
        outputs = new HoodIOOutputs();

        OTM = false;

        hoodStallingDebouncer = new Debouncer(Settings.Superstructure.Hood.STALL_DEBOUNCE, DebounceType.kBoth);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public void periodicAfterScheduler() {
        Logger.recordOutput("Hood/Output Mode", outputs.outputMode);
        Logger.recordOutput("Hood/Position Setpoint", outputs.position);
        Logger.recordOutput("Hood/Voltage Setpoint", outputs.voltage);

        io.applyOutputs(outputs);
    }

    public boolean atTolerance() {
        Angle error = inputs.hoodMotorPosition.minus(outputs.position);

        if (Robot.isReal()) {
            if (OTM) {
                return error.abs(Degrees) < Settings.Superstructure.HOOD_SOTM_TOLERANCE.in(Degrees);
            } else {
                return error.abs(Degrees) < Settings.Superstructure.HOOD_TOLERANCE.in(Degrees);
            }
        } else {
            return error.abs(Degrees) < Settings.Superstructure.HOOD_TOLERANCE.in(Degrees) + 5;
        }
    }

    public Angle getHoodAngle() {
        return inputs.hoodMotorPosition;
    }

    private void runPosition(Angle position, boolean OTM) {
        this.OTM = OTM;

        outputs.outputMode = HoodIOOutputMode.POSITION;
        outputs.position = position;
    }

    private void runVoltage(Voltage voltage) {
        outputs.outputMode = HoodIOOutputMode.VOLTAGE;
        outputs.voltage = voltage;
    }

    private Angle hoodAnalogToInput(CommandGamepad gamepad) {
        double hoodMin = Settings.Superstructure.Hood.Angles.MIN.in(Degrees);
        double hoodMax = Settings.Superstructure.Hood.Angles.MAX.in(Degrees);

        return Degrees.of(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2));
    }

    private boolean isStalling() {
        return hoodStallingDebouncer.calculate(
                inputs.hoodMotorStatorCurrent.gt(Settings.Superstructure.Hood.STALL_CURRENT_LIMIT));
    }

    public Command runHomingUpper() {
        Command runVoltage = run(coroutine -> {
            runVoltage(Settings.Superstructure.Hood.HOOD_HOMING_VOLTAGE);
        })
                .until(this::isStalling).named("Hood stalling");

        Command seedHood = run(coroutine -> io.seedHoodPosition(Settings.Superstructure.Hood.MAX_FROM_HORIZON)).named("Seed hood");
        return(runVoltage.andThen(seedHood).andThen(runStow())).named("Run homing upper");
    }

    public Command runStow() {
        return run(coroutine -> runPosition(Settings.Superstructure.Hood.Angles.STOW, false)).named("Run stow");
    }

    public Command runAnalog(CommandGamepad gamepad) {
        return run(coroutine -> runPosition(hoodAnalogToInput(gamepad), false)).named("Run analog");
    }
}
