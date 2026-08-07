package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import java.util.function.Consumer;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.util.DualDebouncer;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.*;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.RobotState;
import org.wpilib.command3.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends Mechanism {
  private static final Intake instance;

  static {
    switch (Settings.currentMode) {
      case REAL -> instance = new Intake(new IntakeIOTalonFX());

      case SIM -> instance = new Intake(new IntakeIOSim());

      default -> instance = new Intake(new IntakeIO() {});
    }
  }

  public static Intake getInstance() {
    return instance;
  }

  @AutoLogOutput(key = "States/Intake/Pivot")
  private PivotState pivotState;

  @AutoLogOutput(key = "States/Intake/Rollers")
  private RollerState rollerState;

  private final DualDebouncer pivotPositionDebouncer;
  private final Debouncer pivotStallingDebouncer;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final IntakeIOOutputs outputs;

  private Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.outputs = new IntakeIOOutputs();
    this.pivotState = PivotState.STOW;
    this.rollerState = RollerState.STOP;

    this.pivotPositionDebouncer = new DualDebouncer(0.5, 0.1);
    this.pivotStallingDebouncer =
        new Debouncer(Settings.Intake.PIVOT_STALL_DEBOUNCE, DebounceType.kBoth);
  }

  public enum PivotState {
    DEPLOY,
    HOMING,
    DIGEST,
    STOW;
  }

  public enum RollerState {
    INTAKE,
    OUTTAKE,
    STOP;
  }

  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (!Settings.EnabledSubsystems.INTAKE.get()) {
      stopPivot();
      stopRollers();

      return;
    }

    switch (pivotState) {
      case DEPLOY -> {
        if (isPivotBelowPushdownThreshold()) {
          Current pushdownCurrent =
              RobotState.isTeleop()
                  ? Settings.Intake.PUSHDOWN_CURRENT_TELEOP
                  : Settings.Intake.PUSHDOWN_CURRENT_AUTON;

          runPivotTorqueCurrent(pushdownCurrent);
        } else {
          runPivotPosition(Settings.Intake.PIVOT_DEPLOY_ANGLE);
        }
      }

      case HOMING -> {
        if (pivotStalling()) {
          io.seedPivotPosition(Settings.Intake.PIVOT_MIN_ANGLE);
          setPivotState(PivotState.DEPLOY);
        } else {
          runPivotVoltage(Settings.Intake.HOMING_VOLTAGE);
        }
      }
      case DIGEST -> runPivotPosition(Settings.Intake.PIVOT_DIGEST_ANGLE);
      case STOW -> runPivotPosition(Settings.Intake.PIVOT_STOW_ANGLE);
    }

    if (pivotState == PivotState.DEPLOY
        && inputs.pivotMotorPosition.lte(Settings.Intake.THRESHOLD_TO_START_ROLLERS)) {
      switch (rollerState) {
        case INTAKE -> runRollersDutyCycle(1.0);
        case OUTTAKE -> runRollersDutyCycle(-1.0);
        case STOP -> stopRollers();
      }
    } else {
      stopRollers();
    }
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private boolean isPivotBelowPushdownThreshold() {
    return pivotPositionDebouncer.calculate(
        inputs.pivotMotorPosition.lte(Settings.Intake.ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE));
  }

  private boolean pivotStalling() {
    return pivotStallingDebouncer.calculate(
        inputs.pivotMotorStatorCurrent.abs(Amps) > Settings.Intake.PIVOT_STALL_CURRENT.in(Amps));
  }

  private void setPivotState(PivotState state) {
    this.pivotState = state;
  }

  private void setRollerState(RollerState state) {
    this.rollerState = state;
  }

  private void stopPivot() {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.STOP;
  }

  private void stopRollers() {
    outputs.rollerMode = IntakeIO.RollerIOOutputMode.STOP;
  }

  private void runPivotPosition(Angle position) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.POSITION;
    outputs.pivotTargetPosition = position;
  }

  private void runPivotTorqueCurrent(Current torqueCurrent) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.TORQUE_CURRENT;
    outputs.pivotTargetTorqueCurrent = torqueCurrent;
  }

  private void runPivotVoltage(Voltage voltage) {
    outputs.pivotMode = IntakeIO.PivotIOOutputMode.VOLTAGE;
    outputs.pivotTargetVoltage = voltage;
  }

  private void runRollersDutyCycle(double dutyCycle) {
    outputs.rollerMode = IntakeIO.RollerIOOutputMode.DUTY_CYCLE;
    outputs.rollerTargetDutyCycle = dutyCycle;
  }

  public Command deploy() {
    return run(
            coroutine -> {
              setPivotState(PivotState.DEPLOY);
              setRollerState(RollerState.INTAKE);
            })
        .named("Intake Deploy");
  }

  public Command stow() {
    return run(
            coroutine -> {
              setPivotState(PivotState.STOW);
              setRollerState(RollerState.STOP);
            })
        .named("Intake Stow");
  }

  public Command home() {
    return run(coroutine -> setPivotState(PivotState.HOMING)).named("Intake Home");
  }

  public Command digest() {
    return run(
            coroutine -> {
              setPivotState(PivotState.DIGEST);
              setRollerState(RollerState.INTAKE);
            })
        .named("Intake Digest");
  }

  public Command autoDigest() {
    return digest()
        .andThen(Command.waitFor(Seconds.of(0.5)).named("Wait")).andThen(deploy()).andThen(Command.waitFor(Seconds.of(0.5)).named("Wait"))
        .andThen(digest()).andThen(Command.waitFor(Seconds.of(0.5)).named("Wait")).andThen(deploy())
        .andThen(Command.waitFor(Seconds.of(0.5)).named("Wait"))
        .andThen(digest()).andThen(Command.waitFor(Seconds.of(0.5)).named("Wait")).andThen(deploy())
        .named("Intake Auto Digest");
  }

  public Command teleopDigest() {
    return digest()
        .andThen(Command.waitFor(Seconds.of(0.5)).named("Wait"))
        .andThen(deploy())
        .andThen(Command.waitFor(Seconds.of(0.5)).named("Wait"))
        .named("Intake Teleop Digest");
        
  }

  public Command outtake() {
    return run(coroutine -> setRollerState(RollerState.OUTTAKE)).named("Intake Outtake");
  }

  public Command runRollers() {
    return run(coroutine -> setRollerState(RollerState.INTAKE)).named("Intake Run Rollers");
  }

  public Command stopRollersCommand() {
    return run(coroutine -> setRollerState(RollerState.STOP)).named("Intake Stop Rollers");
  }

  public Command seedPivotDeployed() {
    return run(
            coroutine -> {
              io.seedPivotPosition(Settings.Intake.PIVOT_DEPLOY_ANGLE);
              setPivotState(PivotState.DEPLOY);
            })
        //.ignoringDisable(true) TODO: Wait for replacement
        .named("Intake Seed Pivot Deployed");
  }

  public Command seedPivotStowed() {
    return run(
            coroutine -> {
              io.seedPivotPosition(Settings.Intake.PIVOT_STOW_ANGLE);
              setPivotState(PivotState.STOW);
            })
        //.ignoringDisable(true) TODO: Wait for replacement
        .named("Intake Seed Pivot Stowed");
  }
}
