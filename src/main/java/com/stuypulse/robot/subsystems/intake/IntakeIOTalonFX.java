package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class IntakeIOTalonFX extends IntakeIOBase {
    public IntakeIOTalonFX() {
        final TalonFX pivotMotor = new TalonFX(IntakeConstants.Ports.PIVOT_MOTOR, Ports.RIO);
        final TalonFX rollerLeaderMotor = new TalonFX(IntakeConstants.Ports.ROLLER_LEADER_MOTOR, Ports.RIO);
        final TalonFX rollerFollowerMotor = new TalonFX(IntakeConstants.Ports.ROLLER_FOLLOWER_MOTOR, Ports.RIO);
        super(pivotMotor, rollerLeaderMotor, rollerFollowerMotor);
    }
}
