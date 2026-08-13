package com.stuypulse.robot.subsystems.superstructure;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.ParallelGroup;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret.TurretState;

public class Superstructure extends Mechanism {
    private static final Superstructure instance;

    private SuperstructureState cachedState;
    private SuperstructureState state;

    private Timer sotmStoppedTimer;
    private Timer fotmStoppedTimer;

    static {
        instance = new Superstructure();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;

    private Optional<Boolean> shouldStop;

    private Debouncer cachedStateIdleDebouncer;

    public Superstructure(){
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        turret = Turret.getInstance();
    }

    public enum SuperstructureState {
        STOW(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
        MANUAL_OVERRIDE(HoodState.MANUAL_OVERRIDE, ShooterState.MANUAL_OVERRIDE, TurretState.SCORE),
        FERRY(HoodState.FERRY, ShooterState.FERRY, TurretState.FERRY),
        FOTM(HoodState.FOTM, ShooterState.FOTM, TurretState.FOTM),
        REVERSE(HoodState.MANUAL_OVERRIDE, ShooterState.REVERSE, TurretState.SCORE),
        KB(HoodState.KB, ShooterState.KB, TurretState.KB),
        LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER, TurretState.LEFT_CORNER),
        RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER, TurretState.RIGHT_CORNER),
        INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION, TurretState.SCORE),
        AUTO_INTERPOLATION(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
        AUTO_INTERPOLATION_SOTM(HoodState.STOW, ShooterState.SOTM, TurretState.SOTM),
        SOTM(HoodState.SOTM, ShooterState.SOTM, TurretState.SOTM);

        private HoodState hoodState;
        private ShooterState shooterState;
        private TurretState turretState;
        
        private SuperstructureState(HoodState hoodState, ShooterState shooterState, TurretState TurretState) {
            this.hoodState = hoodState;
            this.shooterState = shooterState;
            this.turretState = TurretState;
        }

        public HoodState getHoodState() {
            return hoodState;
        }

        public ShooterState getShooterState() {
            return shooterState;
        }

        public TurretState getTurretState() {
            return turretState;
        }
    }

    public void setState(SuperstructureState state) {
        this.state = state;
        hood.setState(state.getHoodState());
        shooter.setState(state.getShooterState());
        turret.setState(state.getTurretState());
    }

    public SuperstructureState getState() {
        return state;
    }

    public boolean shouldStop() {
        if (!shouldStop.isEmpty()){
            return shouldStop.get();
        }

        Drive swerve = Drive.getInstance();

        boolean isSpindexerStopState = Spindexer.getInstance().getState() == SpindexerState.STOP;
        boolean isHandoffStopState = Handoff.getInstance().getState() == HandoffState.STOP;

        boolean isBehindHubWhileFerrying = state == SuperstructureState.FOTM && swerve.isBehindHub();
        boolean isOutsideAllianceZone =
            Drive.getInstance().isOutsideAllianceZone() && state != SuperstructureState.FOTM;
        boolean isUnderTrench =
            Drive.getInstance().isUnderTrench() && state != SuperstructureState.FOTM;
        boolean inManualState =
            state == SuperstructureState.LEFT_CORNER
            && state == SuperstructureState.RIGHT_CORNER
            && state == SuperstructureState.KB;
        boolean isBehindTower = swerve.isBehindTower() && state == SuperstructureState.SOTM;
        boolean isBtwnOppHubAndWall = swerve.isBtwnOppHubAndWall() && state == SuperstructureState.FOTM;

        boolean turretLaggingSOTM = !turret.atTolerance() && state == SuperstructureState.SOTM;
        boolean turretLaggingFOTM = turret.isTurretLaggingFOTM();

        boolean shouldStop =
        isSpindexerStopState
            || isHandOffStopState
            || (isBehindHubWhileFerrying && !inManualState)
            || isBtwnOppHubAndWall
            || turretLaggingSOTM
            || turretLaggingFOTM
            || (isOutsideAllianceZone && !inManualState)
            || (isUnderTrench && !inManualState)
            || isBehindTower;

        this.shouldStop = Optional.of(shouldStop);

        return shouldStop;
    }

    public boolean isReadyToShoot() {
        return hood.hoodReadyToShoot() && shooter.readyToShoot() && turret.readyToShoot();
    }

    public boolean atTolerance() {
        return hood.atTolerance() && shooter.atTolerance() && turret.atTolerance();
    }

    public void clearMemoized() {
        this.shouldStop = Optional.empty();
    }

    public boolean isHopperEmpty() {
        return !shooter.isShooting();
    }

    public void periodicAfterScheduler() {
        if (state == SuperstructureState.SOTM && shouldStop() && RobotState.isEnabled()) {
            sotmStoppedTimer.start();
        } else if (state == SuperstructureState.FOTM && shouldStop() && RobotState.isEnabled()) {
            fotmStoppedTimer.start();
        }

        if (state != SuperstructureState.SOTM) sotmStoppedTimer.stop();
        if (state != SuperstructureState.FOTM) fotmStoppedTimer.stop();

        if (!shouldStop() || RobotState.isDisabled()) {
            sotmStoppedTimer.stop();
            fotmStoppedTimer.stop();
        }

        if (Drive.getInstance().isOutsideAllianceZone()
            && state == SuperstructureState.SOTM
            && !Robot
            .isAutonomous()) { // allows us to start SOTM earlier in auto, but currently not desired
      // in teleop
            setState(SuperstructureState.STOW);
            Spindexer.getInstance().setState(SpindexerState.STOP);
            Handoff.getInstance().setState(HandoffState.STOP);
        }
  }

    public Command setStateCommand(SuperstructureState state) {
        return run(coroutine -> setState(state)).named("Set State");
    }

    public Command autoInterpolationSOTM() {
        return run(coroutine -> setStateCommand(SuperstructureState.AUTO_INTERPOLATION_SOTM))
        .named("Superstructure Auto Interpolation SOTM");
    }

    public Command FOTM() {
        return run(coroutine -> setStateCommand(SuperstructureState.FOTM)).named("Superstructure FOTM");
    }

    public Command ferry() {
        return run(coroutine -> setStateCommand(SuperstructureState.FERRY)).named("Superstructure Ferry");
    }

    public Command interpolation() {
        return run(coroutine -> setStateCommand(SuperstructureState.INTERPOLATION))
        .named("Superstructure Interpolation");
    }

    public Command kb() {
        return run(coroutine -> setStateCommand(SuperstructureState.KB)).named("Superstructure KB");
    }

    public Command leftCorner() {
        return run(coroutine -> setStateCommand(SuperstructureState.LEFT_CORNER)).named("Superstructure Left Corner");
    }

    public Command manualOverride() {
        return run(coroutine -> setStateCommand(SuperstructureState.MANUAL_OVERRIDE))
        .named("Superstructure Manual Override");
    }

    public Command reverse() {
        return run(coroutine -> setStateCommand(SuperstructureState.REVERSE)).named("Superstructure Reverse");
    }

    public Command rightCorner() {
        return run(coroutine -> setStateCommand(SuperstructureState.RIGHT_CORNER))
        .named("Superstructure Right Corner");
    }

    public Command SOTM() {
        return run(coroutine -> setStateCommand(SuperstructureState.SOTM)).named("Superstructure SOTM");
    }

    public Command stow() {
        return run(coroutine -> setStateCommand(SuperstructureState.STOW)).named("Superstructure Stow");
    }

    private BooleanSupplier calculateCachedStateIdle(CommandGamepad driver) {
        Translation2d driverInputAsVelocity =
            DriveCommands.getLinearVelocityFromJoysticks(
                -driver.getLeftY(), -driver.getLeftX());

        return cachedStateIdleDebouncer.calculate(
            driverInputAsVelocity.getNorm() <= Driver.Drive.DEADBAND
                && Math.abs(driver.getRightX()) <= Turn.DEADBAND);
    }

    public Command cacheState(CommandGamepad driver) {
        Command getDrive = run(
            coroutine -> {
              this.cachedState = state;
              setState(SuperstructureState.INTERPOLATION);
              Drive.getInstance().stopWithX();
            }).named("Get drive");

        Command setState = run(coroutine -> setState(cachedState)).named("Set State");

        ParallelGroup cache = getDrive.alongWith(setState).named("Cache");

        return run(
            coroutine -> 
            cache
            .until(calculateCachedStateIdle(driver)))
            .named("Superstructure Cache State");
  }
}
