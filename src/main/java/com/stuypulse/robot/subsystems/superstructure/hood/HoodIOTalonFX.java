package com.stuypulse.robot.subsystems.superstructure.hood;


import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class HoodIOTalonFX extends HoodIOBase {
    public HoodIOTalonFX() {
        final TalonFX hoodMotor = new TalonFX(Ports.Superstructure.Hood.MOTOR, Ports.RIO);
        super(hoodMotor);
    }
}
