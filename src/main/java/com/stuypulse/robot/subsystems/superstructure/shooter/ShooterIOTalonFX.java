package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class ShooterIOTalonFX extends ShooterIOBase {
    public ShooterIOTalonFX() {
        final TalonFX shooterLeader = new TalonFX(Ports.Superstructure.Shooter.MOTOR_LEAD, Ports.RIO);
        final TalonFX shooterFollower = new TalonFX(Ports.Superstructure.Shooter.MOTOR_FOLLOW, Ports.RIO);
        super(shooterLeader, shooterFollower);
    }
}