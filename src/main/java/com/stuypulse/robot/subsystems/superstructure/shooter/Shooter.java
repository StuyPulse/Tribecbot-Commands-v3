package com.stuypulse.robot.subsystems.superstructure.shooter;

import static org.wpilib.units.Units.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterIO.ShooterIOOutputs;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;

import com.stuypulse.robot.util.FullSubsystem;

import org.wpilib.command3.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {
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

    @AutoLogOutput(key = "States/Shooter")
    private ShooterState state;

    private final Debouncer readyToShootDebouncer;
    private final Debouncer currentlyShootingDebouncer;
    private boolean atTolerance;

    private Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.outputs = new ShooterIOOutputs();

        readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
        currentlyShootingDebouncer = new Debouncer(2, DebounceType.kFalling);
        atTolerance = false;
    }

    public enum ShooterState {
        STOP,
        MANUAL_OVERRIDE,
        FERRY,
        REVERSE,
        KB,
        LEFT_CORNER,
        RIGHT_CORNER,
        INTERPOLATION,
        SOTM,
        FOTM;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
      stopShooter();

      return;
    }

    switch (state) {
      case STOP -> stopShooter();
      case MANUAL_OVERRIDE -> runVelocity(
          RPM.of(SuperstructureConstants.Shooter.Settings.RPM.MANUAL_OVERRIDE.get()));
      case FERRY -> runVelocity(InterpolationCalculator.getInterpolatedFerryRPM());
      case REVERSE -> runVelocity(SuperstructureConstants.Shooter.Settings.RPM.REVERSE);
      case KB -> runVelocity(SuperstructureConstants.Shooter.Settings.RPM.KB);
      case LEFT_CORNER -> runVelocity(SuperstructureConstants.Shooter.Settings.RPM.LEFT_CORNER);
      case RIGHT_CORNER -> runVelocity(SuperstructureConstants.Shooter.Settings.RPM.RIGHT_CORNER);
      case INTERPOLATION -> runVelocity(InterpolationCalculator.getInterpolatedShotRPM());
      case SOTM -> runVelocity(SOTMCalculator.calculateShooterRPMSOTM());
      case FOTM -> runVelocity(SOTMCalculator.calculateShooterRPMFOTM());
    }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    public AngularVelocity getShooterVelocity() {
        return inputs.shooterLeaderMotorVelocity;
    }

    public void stopShooter() {
        outputs.shooterMode = ShooterIOOutputMode.STOP;
    }
    private void runVelocity(AngularVelocity velocity) {
        outputs.shooterMode = ShooterIOOutputMode.VELOCITY;
        outputs.shooterVelocity = velocity;

        AngularVelocity error = inputs.shooterLeaderMotorVelocity.minus(velocity);

        AngularVelocity toleranceHigh =
            switch (state) {
            case SOTM -> RPM.of(SuperstructureConstants.Settings.SHOOTER_FOTM_TOLERANCE_RPM_HIGH);
            case FOTM -> RPM.of(SuperstructureConstants.Settings.SHOOTER_FOTM_TOLERANCE_RPM_HIGH);
            default -> RPM.of(SuperstructureConstants.Settings.SHOOTER_TOLERANCE_RPM_HIGH);
        };

        AngularVelocity toleranceLow =
            switch (state) {
            case SOTM -> RPM.of(SuperstructureConstants.Settings.SHOOTER_SOTM_TOLERANCE_RPM_LOW);
            case FOTM -> RPM.of(SuperstructureConstants.Settings.SHOOTER_FOTM_TOLERANCE_RPM_LOW);
            default -> RPM.of(SuperstructureConstants.Settings.SHOOTER_TOLERANCE_RPM_LOW);
        };

    atTolerance = error.lt(toleranceLow.unaryMinus()) && error.gt(toleranceHigh);    
    }

    public boolean readyToShoot() {
        return readyToShootDebouncer.calculate(atTolerance);
    }

    public boolean atTolerance() {
        return atTolerance;
    }

    public boolean isShooting() {
        return currentlyShootingDebouncer.calculate(
            inputs.shooterLeaderMotorStatorCurrent.gt(
            SuperstructureConstants.Shooter.Settings.IS_SHOOTING_CURRENT));
    }

    private void setState(ShooterState state) {
        this.state = state;
    }

    public void setStateCommand(ShooterState state) {
        setState(state);
    }
}
