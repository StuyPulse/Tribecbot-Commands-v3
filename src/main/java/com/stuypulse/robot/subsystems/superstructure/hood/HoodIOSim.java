package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class HoodIOSim implements HoodIO {

    private final SystemSim<ElevatorSim> sim;
    private static final double HOOD_ARM_LENGTH_METERS = 0.3;

    private static final double MIN_HEIGHT =
      HOOD_ARM_LENGTH_METERS
          * Math.sin((SuperstructureConstants.Hood.Settings.Angles.MIN.in(Radians)));
    private static final double MAX_HEIGHT =
      HOOD_ARM_LENGTH_METERS
          * Math.sin((SuperstructureConstants.Hood.Settings.Angles.MAX.in(Radians)));

    private static final double DRUM_RADIUS = 0.01;

    private final TalonFXSimulation hoodMotor;

    private final PositionVoltage positionController;
    private final VoltageOut homingController;

    private final StatusSignal<Angle> hoodMotorPosition;
    private final StatusSignal<Current> hoodMotorSupplyCurrent;
    private final StatusSignal<Current> hoodMotorStatorCurrnet;
    private final StatusSignal<Temperature> hoodMotorTermperature;
    private final StatusSignal<Voltage> hoodMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> hoodMotorVelocity;

    public HoodIOSim(){
        sim = 
            SystemSim.of(
                new ElevatorSim(
                    Models.elevatorFromPhysicalConstants(
                        DCMotor.getKrakenX60(1), 1.0, DRUM_RADIUS, 1.0), 
                    DCMotor.getKrakenX60(1), 
                    MIN_HEIGHT, 
                    MAX_HEIGHT, 
                    false, 
                    MIN_HEIGHT, 
                    0.001), 
                Meters.of(DRUM_RADIUS));

        hoodMotor = new TalonFXSimulation(Ports.Superstructure.Hood.MOTOR, 1, sim);

        positionController = new PositionVoltage(0).withEnableFOC(true);
        homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);

        hoodMotorPosition = hoodMotor.getPosition();
        hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodMotorStatorCurrnet = hoodMotor.getStatorCurrent();
        hoodMotorTermperature = hoodMotor.getDeviceTemp();
        hoodMotorAppliedVoltage = hoodMotor.getMotorVoltage();
        hoodMotorVelocity = hoodMotor.getVelocity();                    
    }

    @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodMotor.refresh();

    BaseStatusSignal.refreshAll(
        hoodMotorPosition,
        hoodMotorSupplyCurrent,
        hoodMotorStatorCurrnet,
        hoodMotorTermperature,
        hoodMotorAppliedVoltage,
        hoodMotorVelocity);
    inputs.hoodMotorPosition = hoodMotorPosition.getValue();
    inputs.hoodMotorSupplyCurrent = hoodMotorSupplyCurrent.getValue();
    inputs.hoodMotorStatorCurrent = hoodMotorStatorCurrnet.getValue();
    inputs.hoodMotorTemperature = hoodMotorTermperature.getValue();
    inputs.hoodMotorAppliedVoltage = hoodMotorAppliedVoltage.getValue();
    inputs.hoodMotorVelocity = hoodMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.outputMode) {
      case POSITION -> hoodMotor.setControl(positionController.withPosition(outputs.position));

      case VOLTAGE -> hoodMotor.setControl(homingController.withOutput(outputs.voltage));

      case STOP -> hoodMotor.stopMotor();
    }
  }

  @Override
  public void seedHoodPosition(Angle position) {
    hoodMotor.setPosition(position);
  }
}

