package com.stuypulse.robot.subsystems.superstructure.turret;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class Turret extends Mechanism {
    private static final Turret instance;
    private Translation2d driverInput;

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Turret(new TurretIOTalonFX());

            case SIM -> instance = new Turret(new TurretIOSim());

            default -> instance = new Turret(new TurretIO() {
            });
        }
    }

    public static Turret getInstance() {
        return instance;
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final TurretIOOutputs outputs;

    private boolean OTM;
    private boolean atTolerance;

    private final Debouncer readyToShootDebouncer;

    public Turret(TurretIO io) {
        driverInput = Translation2d.kZero;

        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.outputs = new TurretIOOutputs();

        readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
        OTM = false;
        atTolerance = false;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    public boolean atTolerance() {
        Angle error = inputs.turretMotorPosition.minus(outputs.turretPosition);

        if (Robot.isReal()) {
            if (OTM) {
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_HIGH;
            } else {
                return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
            }
        } else {
            return error.abs(Degrees) < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW;
        }
    }

    public boolean turretReadyToShoot() {
        return readyToShootDebouncer.calculate(atTolerance);
    }

    public Rotation2d getTurretYaw() {
        return Rotation2d.fromDegrees(inputs.turretMotorPosition.in(Degrees));
    }

    private void runPosition(Angle position, boolean OTM) {
        this.OTM = OTM;

        outputs.turretPosition = position;
    }

    private Angle driverInputToAngle() {
        Logger.recordOutput("Superstructure/Input/Driver Output", driverInput.getX());
        return Degrees.of(driverInput.getX() * 180);
    }

    public Command runFerry() {
        return run(coroutine -> runPosition(Settings.Superstructure.Turret.FOTM_TOLERANCE, OTM)).named("Run ferry");
    }

    public Command runLeftCorner() {
        return run(coroutine -> runPosition(Settings.Superstructure.Turret.LEFT_CORNER, OTM)).named("Run left corner");
    }

    public Command runRightCorner() {
        return run(coroutine -> runPosition(Settings.Superstructure.Turret.RIGHT_CORNER, OTM)).named("Run right corner");
    }

    public Command runKB() {
        return run(coroutine -> runPosition(Settings.Superstructure.Turret.KB, OTM)).named("Run KB");
    }

    public Command runShoot() {
        return run(
                coroutine -> runPosition(
                        Degrees.of(Settings.Superstructure.Turret.SOTM_TOLERANCE_FAR.getAsDouble()), OTM)).named("Run shoot");
    }

    public Command runIdle() {
        return run(coroutine -> runPosition(Degrees.of(0), OTM)).named("Run idle");
    }

    public Command runAnalog(CommandGamepad gamepad) {
        return run(coroutine -> runPosition(driverInputToAngle(), OTM)).named("Run analog");
    }
}
