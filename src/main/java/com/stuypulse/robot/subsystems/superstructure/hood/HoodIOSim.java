package com.stuypulse.robot.subsystems.superstructure.hood;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.SuperstructureConstants;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class HoodIOSim extends HoodIOBase {

    private final SystemSim<ElevatorSim> sim;

    private final TalonFXSimulation hoodMotor;

    private final PositionVoltage positionController;
    private final VoltageOut homingController;

    public HoodIOSim(){
        final SystemSim<ElevatorSim> sim = 
            SystemSim.of(
                new ElevatorSim(
                    Models.elevatorFromPhysicalConstants(
                        DCMotor.getKrakenX60(1), 1.0, SuperstructureConstants.Hood.Settings.DRUM_RADIUS, 1.0), 
                    DCMotor.getKrakenX60(1), 
                    SuperstructureConstants.Hood.Settings.MIN_HEIGHT, 
                    SuperstructureConstants.Hood.Settings.MAX_HEIGHT, 
                    false, 
                    SuperstructureConstants.Hood.Settings.MIN_HEIGHT, 
                    0.001), 
                Meters.of(SuperstructureConstants.Hood.Settings.DRUM_RADIUS));

        final TalonFXSimulation hoodMotor = new TalonFXSimulation(Ports.Superstructure.Hood.MOTOR, 1, sim);
        
        super(hoodMotor);

        this.hoodMotor = hoodMotor;
        this.sim = sim;

        positionController = new PositionVoltage(0).withEnableFOC(true);
        homingController = new VoltageOut(0).withIgnoreSoftwareLimits(true);
    }

    @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodMotor.refresh();

    super.updateInputs(inputs);
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

