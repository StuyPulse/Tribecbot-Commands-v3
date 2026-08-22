package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class HandoffIOTalonFX extends HandoffIOBase {
    public HandoffIOTalonFX() {
        final TalonFX handoffLeaderMotor = new TalonFX(HandoffConstants.Ports.LEADER_MOTOR, Ports.RIO);
        final TalonFX handoffFollowerMotor = new TalonFX(HandoffConstants.Ports.FOLLOWER_MOTOR, Ports.RIO);
        super(handoffLeaderMotor, handoffFollowerMotor);
    }
}
