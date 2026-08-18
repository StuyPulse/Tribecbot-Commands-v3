// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.swerve.Drive;

import org.wpilib.command3.*;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

import static org.wpilib.units.Units.Seconds;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands extends Mechanism{
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive.run(
        coroutine -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisVelocities speeds =
              new ChassisVelocities(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              MatchState.getAlliance().isPresent()
                  && MatchState.getAlliance().get() == Alliance.RED;
          drive.runVelocity(
              speeds.toFieldRelative(
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        }).named("Joystick Drive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    Command joystickDrive = drive.run(
            coroutine -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisVelocities speeds =
                  new ChassisVelocities(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              drive.runVelocity(
                  speeds.toFieldRelative(
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            }).named("Joystick Drive at Angle");

        // Reset PID controller when command starts
        Command resetAngleController = drive.run(
            coroutine -> angleController.reset(drive.getRotation().getRadians())).named("Reset angle controller");

        return drive.run(coroutine -> resetAngleController.andThen(joystickDrive)).named("Joystick Drive at Angle");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    Command resetData = drive.run(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }).named("Reset data");

    Command orientModules = drive.run(coroutine -> 
        drive.runCharacterization(0.0)).named("Orient modules").withTimeout(Seconds.of(FF_START_DELAY));

    Command startTimer = drive.run(coroutine -> timer.restart()).named("Start timer");

    Command accelerateAndGatherData = drive.run(
            coroutine -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            }).named("Accelerate and gather data");

    Command calculate = drive.run(
            coroutine -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            }).named("Calculate results");

    return resetData.andThen(orientModules).andThen(startTimer)
        .andThen(accelerateAndGatherData).andThen(calculate).named("Joystick Drive Angle");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    Command resetLimiter = drive.run(coroutine -> limiter.reset(0.0)).named("Reset limiter");

    Command accelerate = drive.run(coroutine -> {
      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
      drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
    }).named("Accelerate");

    Command recordStartingMeasurement = drive.run(coroutine -> {
      state.positions = drive.getWheelRadiusCharacterizationPositions();
      state.lastAngle = drive.getRotation();
      state.gyroDelta = 0.0;
    }).named("Record starting measurement");

    Command updateGyroDelta = drive.run(coroutine -> {
      var rotation = drive.getRotation();
      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
      state.lastAngle = rotation;
    }).named("Update gyro delta");

    Command calculate = drive.run(coroutine -> {
      double[] positions = drive.getWheelRadiusCharacterizationPositions();
      double wheelDelta = 0.0;
      for (int i = 0; i < 4; i++) {
        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
      }
      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

      NumberFormat formatter = new DecimalFormat("#0.000");
      System.out.println("********** Wheel Radius Characterization Results **********");
      System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
      System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
      System.out.println(
          "\tWheel Radius: "
              + formatter.format(wheelRadius)
              + " meters, "
              + formatter.format(Units.metersToInches(wheelRadius))
              + " inches");
    }).named("Calculate results");

    Command driveControl = resetLimiter.andThen(accelerate).named("Drive control");
    Command measurement = Command.waitFor(Seconds.of(1.0)).named("Wait 1 sec")
        .andThen(recordStartingMeasurement).andThen(updateGyroDelta).named("Measurement");
	
	Command wheel = driveControl.alongWith(measurement).named("Control and measure");

    return wheel.andThen(calculate).named("Wheel radius characterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
