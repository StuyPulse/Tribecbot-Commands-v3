package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.Simulation.TalonFXSimulation.TalonFXSimulation;

public class IntakeIOSim extends IntakeIOBase {
    private final SystemSim<SingleJointedArmSim> pivotSim;
    private final TalonFXSimulation pivotMotor;

    private final SystemSim<FlywheelSim> rollerSim;
    private final TalonFXSimulation rollerLeaderMotor;
    private final TalonFXSimulation rollerFollowerMotor;

    public IntakeIOSim() {
        final double pivotGearRatio = IntakeConstants.Intake.PIVOT_GEAR_RATIO;
        final SystemSim<SingleJointedArmSim> pivotSim = SystemSim.of(new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1), 
            pivotGearRatio, 
            IntakeConstants.Intake.PIVOT_MOI.in(KilogramSquareMeters), 
            IntakeConstants.Intake.PIVOT_ARM_LENGTH.in(Meters), 
            IntakeConstants.Intake.PIVOT_MIN_ANGLE.in(Radians), 
            IntakeConstants.Intake.PIVOT_MAX_ANGLE.in(Radians), 
            true, 
            IntakeConstants.Intake.PIVOT_STOW_ANGLE.in(Radians)
        ));
        final TalonFXSimulation pivotMotor = new TalonFXSimulation(Ports.Intake.PIVOT, pivotGearRatio, pivotSim);

        final double rollerGearRatio = IntakeConstants.Intake.ROLLER_GEAR_RATIO;
        final SystemSim<FlywheelSim> rollerSim = SystemSim.of(new FlywheelSim(Models.flywheelFromPhysicalConstants(
            DCMotor.getKrakenX60Foc(2),
            IntakeConstants.Intake.ROLLER_MOI.in(KilogramSquareMeters),
            rollerGearRatio
        ), DCMotor.getKrakenX60Foc(2)));
        final TalonFXSimulation rollerLeaderMotor = new TalonFXSimulation(Ports.Intake.ROLLER_LEADER, rollerGearRatio, rollerSim);
        final TalonFXSimulation rollerFollowerMotor = new TalonFXSimulation(Ports.Intake.ROLLER_FOLLOWER, rollerGearRatio, rollerSim);

        super(pivotMotor, rollerLeaderMotor, rollerFollowerMotor);

        this.pivotSim = pivotSim;
        this.pivotMotor = pivotMotor;

        this.rollerSim = rollerSim;
        this.rollerLeaderMotor = rollerLeaderMotor;
        this.rollerFollowerMotor = rollerFollowerMotor;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivotSim.update(Settings.DT);
        pivotMotor.refresh();

        rollerSim.update(Settings.DT);
        rollerLeaderMotor.refresh();
        rollerFollowerMotor.refresh();

        super.updateInputs(inputs);
    }
}
