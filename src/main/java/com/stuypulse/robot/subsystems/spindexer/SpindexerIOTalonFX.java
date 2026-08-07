package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class SpindexerIOTalonFX extends SpindexerIOBase {
    public SpindexerIOTalonFX() {
        final TalonFX spindexerLeaderMotor = new TalonFX(Ports.Spindexer.LEADER, Ports.RIO);
        final TalonFX spindexerFollowerMotor = new TalonFX(Ports.Spindexer.FOLLOWER, Ports.RIO);
        super(spindexerLeaderMotor, spindexerFollowerMotor);
    }
}
