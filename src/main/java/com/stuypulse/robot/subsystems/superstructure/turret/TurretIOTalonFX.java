package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.stuypulse.robot.constants.Motors.CANCoderConfig;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.*;

public class TurretIOTalonFX implements TurretIO {
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

    public TurretIOTalonFX() {
        turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);

        encoder17t = new CANcoder(Ports.Superstructure.Turret.ENCODER17T, Ports.RIO);
        encoder18t = new CANcoder(Ports.Superstructure.Turret.ENCODER18T, Ports.RIO);

        positionController = new PositionVoltage(0).withEnableFOC(true);

        
        encoder17tConfig =  
            new CANCoderConfig()
              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
              .withMagnetOffset(SuperstructureConstants.Turret.Settings.Encoder17t.OFFSET.in(Rotations))
              .withAbsoluteSensorDiscontinuityPoint(1.0);
        encoder18tConfig = 
            new CANCoderConfig()
              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
              .withMagnetOffset(SuperstructureConstants.Turret.Settings.Encoder18t.OFFSET.in(Rotations))
              .withAbsoluteSensorDiscontinuityPoint(1.0);

        turretMotorPosition = turretMotor.getPosition();
        turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();
        turretMotorStatorCurrent = turretMotor.getStatorCurrent();
        turretMotorTemperature = turretMotor.getDeviceTemp();
        turretMotorAppliedVoltage = turretMotor.getMotorVoltage();
        turretMotorVelocity = turretMotor.getVelocity();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                turretMotorPosition,
                turretMotorSupplyCurrent,
                turretMotorStatorCurrent,
                turretMotorTemperature,
                turretMotorAppliedVoltage,
                turretMotorVelocity);
        inputs.turretMotorPosition = turretMotorPosition.getValue();
        inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValue();
        inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValue();
        inputs.turretMotorTemperature = turretMotorTemperature.getValue();
        inputs.turretMotorAppliedVoltage = turretMotorAppliedVoltage.getValue();
        inputs.turretMotorVelocity = turretMotorVelocity.getValue();
    }

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
        turretMotor.setControl(positionController.withPosition(outputs.turretPosition));
    }
}
