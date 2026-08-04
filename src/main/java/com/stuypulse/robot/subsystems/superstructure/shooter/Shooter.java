package com.stuypulse.robot.subsystems.superstructure.shooter;

import static org.wpilib.units.Units.*;
import org.wpilib.units.measure.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputs;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Mechanism {
    private static final Shooter instance;

    static {
        switch (Settings.currentMode) {
            case REAL -> instance = new Shooter(new ShooterIOTalonFX());

            case SIM -> instance = new Shooter(new ShooterIOSim());

            default -> instance = new Shooter(new ShooterIO() {
            });
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;
    private final ShooterIOOutputs outputs;

    private final Debouncer readyToShootDebouncer;
    private boolean atTolerance;

    private Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.outputs = new ShooterIOOutputs();

        readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
        atTolerance = false;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void periodicAfterScheduler() {
        Logger.recordOutput("Shooter/Velocity Setpoint", outputs.shooterVelocity);
        io.applyOutputs(outputs);
    }

    public AngularVelocity getShooterVelocity() {
        return inputs.shooterLeaderMotorVelocity;
    }

    private void runVelocity(AngularVelocity velocity) {
        outputs.shooterVelocity = velocity;
    }

    public boolean readyToShoot() {
        return readyToShootDebouncer.calculate(atTolerance);
    }

    public Command stopShooter() {
        return run(coroutine -> runVelocity(RPM.zero())).named("Stop shooter");
    }

    // Anything that isn't SOTM or FOTM
    private Command runManual(DoubleSupplier rpmSupplier) {
        return run(
                coroutine -> {
                    double targetRPM = rpmSupplier.getAsDouble();

                    runVelocity(RPM.of(targetRPM));
                    double error = inputs.shooterLeaderMotorVelocity.in(RPM) - targetRPM;

                    atTolerance = error > -Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW
                            && error < Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
                }).named("Run manual");
    }

    public Command runManualOverride() {
        return runManual(Settings.Superstructure.Shooter.RPM.MANUAL_OVERRIDE::get);
    }

    public Command runLeftCorner() {
        return runManual(() -> Settings.Superstructure.Shooter.RPM.LEFT_CORNER.in(RPM));
    }

    public Command runRightCorner() {
        return runManual(() -> Settings.Superstructure.Shooter.RPM.RIGHT_CORNER.in(RPM));
    }

    public Command runReverse() {
        return runManual(() -> Settings.Superstructure.Shooter.RPM.REVERSE.in(RPM));
    }
}
