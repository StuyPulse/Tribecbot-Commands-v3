package com.stuypulse.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class TurretIOTalonFX extends TurretIOBase {
    public TurretIOTalonFX() {
        final TalonFX turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);
        final CANcoder encoder17t = new CANcoder(Ports.Superstructure.Turret.ENCODER17T, Ports.RIO);
        final CANcoder encoder18t = new CANcoder(Ports.Superstructure.Turret.ENCODER18T, Ports.RIO);
        super(turretMotor, encoder17t, encoder18t);
    }
}
