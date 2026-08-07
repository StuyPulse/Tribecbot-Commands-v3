package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class IntakeIOTalonFX extends IntakeIOBase {
    public IntakeIOTalonFX() {
        final TalonFX pivotMotor = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        final TalonFX rollerLeaderMotor = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        final TalonFX rollerFollowerMotor = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        super(pivotMotor, rollerLeaderMotor, rollerFollowerMotor);
    }
}
