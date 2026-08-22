/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.stuypulse.robot.constants.Motors.CANCoderConfig;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;

public abstract class TurretIOBase implements TurretIO {
  private final TalonFX turretMotor;

  private final CANcoder encoder17t;
  private final CANcoder encoder18t;

  private CANCoderConfig encoder17tConfig;
  private CANCoderConfig encoder18tConfig;

  private final PositionVoltage positionController;

  private final StatusSignal<Angle> turretMotorPosition;
  private final StatusSignal<Current> turretMotorSupplyCurrent;
  private final StatusSignal<Current> turretMotorStatorCurrent;
  private final StatusSignal<Temperature> turretMotorTemperature;
  private final StatusSignal<Voltage> turretMotorAppliedVoltage;
  private final StatusSignal<AngularVelocity> turretMotorVelocity;

  private final StatusSignal<Angle> encoder17tPosition;
  private final StatusSignal<Angle> encoder18tPosition;

  public TurretIOBase(TalonFX turretMotor, CANcoder encoder17t, CANcoder encoder18t) {
    this.turretMotor = turretMotor;
    this.encoder17t = encoder17t;
    this.encoder18t = encoder18t;

    SuperstructureConstants.Turret.Motors.TURRET_CONFIG.configure(turretMotor);

    turretMotor.getClosedLoopError().setUpdateFrequency(Hertz.of(50));

    positionController = new PositionVoltage(0).withEnableFOC(true);

    encoder17tConfig =
        new CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(
                SuperstructureConstants.Turret.Settings.Encoder17t.OFFSET.in(Rotations))
            .withAbsoluteSensorDiscontinuityPoint(1.0);
    encoder18tConfig =
        new CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(
                SuperstructureConstants.Turret.Settings.Encoder18t.OFFSET.in(Rotations))
            .withAbsoluteSensorDiscontinuityPoint(1.0);

    turretMotorPosition = turretMotor.getPosition();
    turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();
    turretMotorStatorCurrent = turretMotor.getStatorCurrent();
    turretMotorTemperature = turretMotor.getDeviceTemp();
    turretMotorAppliedVoltage = turretMotor.getMotorVoltage();
    turretMotorVelocity = turretMotor.getVelocity();

    encoder17tPosition = encoder17t.getAbsolutePosition();
    encoder18tPosition = encoder18t.getAbsolutePosition();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretMotorPosition,
        turretMotorSupplyCurrent,
        turretMotorStatorCurrent,
        turretMotorTemperature,
        turretMotorAppliedVoltage,
        turretMotorVelocity,
        encoder17tPosition,
        encoder18tPosition);
    inputs.turretMotorPosition = turretMotorPosition.getValue();
    inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValue();
    inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValue();
    inputs.turretMotorTemperature = turretMotorTemperature.getValue();
    inputs.turretMotorAppliedVoltage = turretMotorAppliedVoltage.getValue();
    inputs.turretMotorVelocity = turretMotorVelocity.getValue();

    inputs.encoder17tPosition = encoder17tPosition.getValue();
    inputs.encoder18tPosition = encoder18tPosition.getValue();

    inputs.encoder17tMagnetOffset = encoder17tConfig.getConfiguration().MagnetSensor.MagnetOffset;
    inputs.encoder18tMagnetOffset = encoder18tConfig.getConfiguration().MagnetSensor.MagnetOffset;
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.turretMode) {
      case POSITION ->
          turretMotor.setControl(
              positionController
                  .withPosition(outputs.turretPosition)
                  .withSlot(outputs.gainSlot)
                  .withFeedForward(outputs.feedForward));

      case STOP -> turretMotor.stopMotor();
    }
  }

  @Override
  public void seedTurretPosition(Angle position) {
    turretMotor.setPosition(position);
  }

  @Override
  public void refreshMagnetSensorConfigs() {
    encoder17t.getConfigurator().refresh(encoder17tConfig.getConfiguration().MagnetSensor);
    encoder18t.getConfigurator().refresh(encoder18tConfig.getConfiguration().MagnetSensor);
  }

  @Override
  public void reconfigureEncoderMagnetOffsets(double offset17t, double offset18t) {
    encoder17tConfig.withMagnetOffset(offset17t);
    encoder18tConfig.withMagnetOffset(offset18t);

    encoder17tConfig.configure(encoder17t);
    encoder18tConfig.configure(encoder18t);
  }
}
