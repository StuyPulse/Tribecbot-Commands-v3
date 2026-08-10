package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class SpindexerIOTalonFX extends SpindexerIOBase {
    public SpindexerIOTalonFX() {
        final TalonFX spindexerLeaderMotor = new TalonFX(SpindexerConstants.Ports.LEADER_MOTOR, Ports.RIO);
        final TalonFX spindexerFollowerMotor = new TalonFX(SpindexerConstants.Ports.FOLLOWER_MOTOR, Ports.RIO);
        super(spindexerLeaderMotor, spindexerFollowerMotor);
    }
}
