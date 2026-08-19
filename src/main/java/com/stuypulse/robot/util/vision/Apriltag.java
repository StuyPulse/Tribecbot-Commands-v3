/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.vision;

import org.wpilib.math.geometry.Pose3d;

public record Apriltag(int id, Pose3d location) {
    public Apriltag(int id, Pose3d location) {
        this.id = id;
        this.location = location;
    }

    public int getID() {
        return id;
    }

    public Pose3d getLocation() {
        return location;
    }
}