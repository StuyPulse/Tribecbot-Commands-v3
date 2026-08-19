package com.stuypulse.robot.subsystems.superstructure.turret;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;;

public class TurretIOSim extends TurretIOBase {

    private SystemSim<DCMotorSim> sim;
    private PositionVoltage controller;
    private TalonFXSimulation simMotor;

    public TurretIOSim(){
        final SystemSim<DCMotorSim> sim = SystemSim.of(
            new DCMotorSim(
                Models.singleJointedArmFromPhysicalConstants(DCMotor.getKrakenX60(1), 
                0, 
                SuperstructureConstants.Turret.Settings.GEAR_RATIO_MOTOR_TO_MECH), 
                DCMotor.getKrakenX60(1), 
            2.8)
        );

        final TalonFXSimulation simMotor =
        new TalonFXSimulation(
            Ports.Superstructure.Turret.MOTOR,
            SuperstructureConstants.Turret.Settings.GEAR_RATIO_MOTOR_TO_MECH,
            sim);
        
        controller = new PositionVoltage(0).withEnableFOC(true);

        super(simMotor, null, null);

        this.simMotor = simMotor;
        this.sim = sim;
    }

    @Override
  public void updateInputs(TurretIOInputs inputs) {
    simMotor.refresh();

    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    simMotor.setControl(controller.withPosition(outputs.turretPosition));
  }
}
