package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.talonfx.sim.SystemSim;
import com.stuypulse.robot.util.talonfx.sim.TalonFXSimulation;

public class IntakeIOSim extends IntakeIOBase {
    private final SystemSim<SingleJointedArmSim> pivotSim;
    private final TalonFXSimulation pivotMotor;

    private final SystemSim<FlywheelSim> rollerSim;
    private final TalonFXSimulation rollerLeaderMotor;
    private final TalonFXSimulation rollerFollowerMotor;

    public IntakeIOSim() {
        final double pivotGearRatio = IntakeConstants.Settings.Pivot.PIVOT_GEAR_RATIO;
        final SystemSim<SingleJointedArmSim> pivotSim = SystemSim.of(new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1), 
            pivotGearRatio, 
            IntakeConstants.Settings.Pivot.PIVOT_MOI.in(KilogramSquareMeters), 
            IntakeConstants.Settings.Pivot.PIVOT_ARM_LENGTH.in(Meters), 
            IntakeConstants.Settings.Pivot.PIVOT_MIN_ANGLE.in(Radians), 
            IntakeConstants.Settings.Pivot.PIVOT_MAX_ANGLE.in(Radians), 
            true, 
            IntakeConstants.Settings.Pivot.PIVOT_STOW_ANGLE.in(Radians)
        ));
        final TalonFXSimulation pivotMotor = new TalonFXSimulation(IntakeConstants.Ports.PIVOT_MOTOR, pivotGearRatio, pivotSim);

        final double rollerGearRatio = IntakeConstants.Settings.Roller.ROLLER_GEAR_RATIO;
        final SystemSim<FlywheelSim> rollerSim = SystemSim.of(new FlywheelSim(Models.flywheelFromPhysicalConstants(
            DCMotor.getKrakenX60Foc(2),
            IntakeConstants.Settings.Roller.ROLLER_MOI.in(KilogramSquareMeters),
            rollerGearRatio
        ), DCMotor.getKrakenX60Foc(2)));
        final TalonFXSimulation rollerLeaderMotor = new TalonFXSimulation(IntakeConstants.Ports.ROLLER_LEADER_MOTOR, rollerGearRatio, rollerSim);
        final TalonFXSimulation rollerFollowerMotor = new TalonFXSimulation(IntakeConstants.Ports.ROLLER_FOLLOWER_MOTOR, rollerGearRatio, rollerSim);

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
