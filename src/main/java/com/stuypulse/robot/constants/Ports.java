/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public CANBus RIO = new CANBus("rio");
    public CANBus CANIVORE = new CANBus("CANIVORE");

    public interface Superstructure {
        public interface Hood {
            int MOTOR = 45;
            int THROUGHBORE_ENCODER = 44;
        }

        public interface Shooter {
            int MOTOR_LEAD = 47;
            int MOTOR_FOLLOW = 46;
        }

        public interface Turret {
            int MOTOR = 40;
            int ENCODER17T = 42;
            int ENCODER18T = 41;
        }
    }
}
