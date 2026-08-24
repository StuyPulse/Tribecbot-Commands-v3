/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretConstants.*;
import com.stuypulse.robot.constants.DriverConstants.*;

import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputMode;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretIO.TurretIOOutputs;
import com.stuypulse.robot.subsystems.swerve.Drive;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private static final Turret instance;
  private Angle driverInput;

  static {
    switch (GlobalSettings.CURRENT_MODE) {
      case REAL -> instance = new Turret(new TurretIOTalonFX());

      case SIM -> instance = new Turret(new TurretIOSim());

      default -> instance = new Turret(new TurretIO() {});
    }
  }

  public static Turret getInstance() {
    return instance;
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs;
  private final TurretIOOutputs outputs;

  @AutoLogOutput(key = "States/Turret")
  private TurretState state;

  private boolean atTolerance;
  private boolean lagging;
  private boolean hasUsedAbsoluteEncoder;
  private boolean hasInitializedFilter;
  private boolean isWrapping;

  private double prevActualTargetAngle;

  private final Debouncer readyToShootDebouncer;

  public Turret(TurretIO io) {
    this.io = io;
    this.inputs = new TurretIOInputsAutoLogged();
    this.outputs = new TurretIOOutputs();

    setState(TurretState.SCORE);

    readyToShootDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    atTolerance = false;

    prevActualTargetAngle = getScoringAngle().in(Degrees);

    hasUsedAbsoluteEncoder = false;
    hasInitializedFilter = false;
  }

  public enum TurretState {
    IDLE,
    ZERO,
    SCORE,
    SOTM,
    FOTM,
    FERRY,
    LEFT_CORNER,
    RIGHT_CORNER,
    KB,
    TESTING;
  }

  private Angle getScoringAngle() {
    Drive swerve = Drive.getInstance();

    Translation2d target = Field.HUB_CENTER.getTranslation();
    Translation2d turret = swerve.getTurretPose().getTranslation();

    return TurretAngleCalculator.getPointAtTargetAngle(target, turret, swerve.getRotation());
  }

  private Angle getFerryAngle() {
    Drive swerve = Drive.getInstance();

    Pose2d robot = swerve.getPose();
    Translation2d target = Field.getFerryZonePose(robot.getTranslation()).getTranslation();
    Translation2d turret = swerve.getTurretPose().getTranslation();

    return TurretAngleCalculator.getPointAtTargetAngle(target, turret, robot.getRotation());
  }

  private double getWrappedTargetAngle(Angle targetAngle) {
    double currentAngle = inputs.turretMotorPosition.in(Degrees);
    return currentAngle + getDelta(targetAngle.in(Degrees), currentAngle);
  }

  private double getDelta(double target, double current) {
    double delta = (target - current) % 360;

    if (delta > 180.0) {
      delta -= 360;
    } else if (delta < -180) {
      delta += 360;
    }

    if (current + delta > TurretSettings.RANGE_CW) {
      return delta - 360;
    }
    if (current + delta < TurretSettings.RANGE_CCW) {
      return delta + 360;
    }

    return delta;
  }

  @AutoLogOutput(key = "Superstructure/Turret/Absolute Angle")
  private Angle getVectorSpaceAngle() {
    return TurretAngleCalculator.getAbsoluteAngle(
        inputs.encoder17tPosition.in(Degrees), inputs.encoder18tPosition.in(Degrees));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (!GlobalSettings.EnabledSubsystems.TURRET.get()) {
      stopTurret();

      return;
    }

    switch (state) {
      case IDLE -> runPosition(inputs.turretMotorPosition);
      case ZERO -> runPosition(Degrees.zero());
      case SCORE -> runPosition(getScoringAngle());
      case SOTM -> runPosition(SOTMCalculator.calculateTurretAngleSOTM());
      case FOTM -> runPosition(SOTMCalculator.calculateTurretAngleFOTM());
      case FERRY -> runPosition(getFerryAngle());
      case LEFT_CORNER -> runPosition(TurretAngles.LEFT_CORNER);
      case RIGHT_CORNER -> runPosition(TurretAngles.RIGHT_CORNER);
      case KB -> runPosition(TurretAngles.KB);
      case TESTING -> runPosition(driverInput);
    }
    ;
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Superstructure/Turret/Ready To Shoot")
  public boolean readyToShoot() {
    return readyToShootDebouncer.calculate(atTolerance);
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public Rotation2d getTurretYaw() {
    return new Rotation2d(inputs.turretMotorPosition);
  }

  public boolean isWrapping() {
    return isWrapping;
  }

  public boolean isTurretLaggingFOTM() {
    return lagging && state == TurretState.FOTM;
  }

  private void seedTurret() {
    io.seedTurretPosition(getVectorSpaceAngle());
  }

  private void setDriverInput(CommandGamepad gamepad) {
    driverInput = Degrees.of(gamepad.getLeftX() * 180);
  }

  private void setState(TurretState state) {
    this.state = state;
  }

  public void setStateCommand(TurretState state) {
    setState(state);
  }

  private void stopTurret() {
    outputs.turretMode = TurretIOOutputMode.STOP;
  }

  private void runPosition(Angle position) {
    if (!hasUsedAbsoluteEncoder) {
      seedTurret();
      hasUsedAbsoluteEncoder = true;
    }

    double currentAngle = inputs.turretMotorPosition.in(Degrees);
    double actualTargetAngle = currentAngle + getDelta(position.in(Degrees), currentAngle);

    if (!hasInitializedFilter) {
      prevActualTargetAngle = actualTargetAngle;
      hasInitializedFilter = true;
    }

    double delta = actualTargetAngle - prevActualTargetAngle;

    boolean deltaIsSignificant =
        Math.abs(delta) >= TurretSettings.SETPOINT_FILTER_THRESHOLD_DEG;

    boolean driverIsMoving =
        Math.abs(RobotContainer.driver.getLeftX()) > DriverDriveSettings.DEADBAND
            || Math.abs(RobotContainer.driver.getLeftY()) > DriverDriveSettings.DEADBAND
            || Math.abs(RobotContainer.driver.getRightX()) > DriverDriveSettings.DEADBAND;

    if (deltaIsSignificant || driverIsMoving) {
      prevActualTargetAngle = actualTargetAngle;
    }

    if (isWrapping) {
      isWrapping =
          Math.abs(getWrappedTargetAngle(position) - currentAngle)
              > TurretSettings.GAIN_SWITCHING_THRESHOLD_END.in(Degrees);
    } else {
      isWrapping =
          Math.abs(getWrappedTargetAngle(position) - currentAngle)
              > TurretSettings.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
    }

    int slot = 0;

    if (isWrapping) {
      slot = 1;
    }

    double omega = Drive.getInstance().getChassisSpeeds().omega;
    double omegaFF = TurretGains.kOmega.get() * omega;
    double setpointVelocityRPS = delta / (360 * 0.02);

    double translationalComponentVelocityRPS = setpointVelocityRPS - omega / (2 * Math.PI);
    double translationFF =
        TurretGains.kTranslation.get() * translationalComponentVelocityRPS;

    outputs.turretMode = TurretIOOutputMode.POSITION;
    outputs.turretPosition = Degrees.of(prevActualTargetAngle);
    outputs.gainSlot = slot;
    outputs.feedForward = omegaFF + translationFF;

    Angle error = inputs.turretMotorPosition.minus(outputs.turretPosition);
    Drive swerve = Drive.getInstance();

    Angle tolerance =
        switch (state) {
          case SOTM ->
              swerve.getTurretPose().getTranslation().getDistance(Field.HUB_CENTER.getTranslation())
                      > TurretSettings.SOTM_TOLERANCE_THRESHOLD_METERS
                          .get()
                  ? Degrees.of(TurretSettings.SOTM_TOLERANCE_CLOSE.get())
                  : Degrees.of(TurretSettings.SOTM_TOLERANCE_FAR.get());
          case FOTM -> TurretSettings.FOTM_TOLERANCE;
          default -> TurretSettings.TOLERANCE;
        };

    atTolerance = error.abs(Degrees) < tolerance.in(Degrees);
    lagging =
        error.abs(Degrees)
            >= TurretSettings.GAIN_SWITCHING_THRESHOLD_START.in(Degrees);
  }

  public Command runFerry() {
    return run(coroutine -> setState(TurretState.FERRY)).named("Run ferry");
  }

  public Command runLeftCorner() {
    return run(coroutine -> setState(TurretState.LEFT_CORNER)).named("Run left corner");
  }

  public Command runRightCorner() {
    return run(coroutine -> setState(TurretState.RIGHT_CORNER)).named("Run right corner");
  }

  public Command runKB() {
    return run(coroutine -> setState(TurretState.KB)).named("Run KB");
  }

  public Command runScore() {
    return run(coroutine -> setState(TurretState.SCORE)).named("Run score");
  }

  public Command runIdle() {
    return run(coroutine -> setState(TurretState.IDLE)).named("Run idle");
  }

  public Command zeroTurret() {
    Command zeroEncoders = run(coroutine -> io.zeroEncoders()).named("Zero encoders");
    Command seedTurret = run(coroutine -> seedTurret()).named("Seed turret");

    return run(coroutine -> zeroEncoders.andThen(seedTurret)).named("Zero turret");
  }

  public Command runAnalog(CommandGamepad gamepad) {
    Command runTesting = run(coroutine -> setState(TurretState.TESTING)).named("Run testing");

    Command driverInput = run(coroutine -> setDriverInput(gamepad)).named("Set driver input");
    return run(coroutine -> runTesting.andThen(driverInput)).named("Run analog");
  }
}
