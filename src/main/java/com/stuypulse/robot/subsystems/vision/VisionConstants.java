
package com.stuypulse.robot.subsystems.vision;

import org.wpilib.math.geometry.Transform3d;

public interface VisionConstants {
    public interface VisionSettings {
        int RESET_IMU_INDEX = 1;

        // Basic filtering thresholds
        double MAX_AMBIGUITY = 0.3;
        double MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
        double ANGULAR_STD_DEV_MEGATAG_2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available

        double BUZZ_DEBOUNCE = 0.25;
    }

    record CameraData(String name, Transform3d robotToCamera, double stdDevFactor) {}

    public enum Cameras {
        // placeholders
        FRONT("Front", new Transform3d(), 1.0),
        BACK("Back", new Transform3d(), 1.0);

        private final CameraData data;

        private Cameras(String name, Transform3d robotToCamera, double stdDevFactor) {
            this.data = new CameraData(name, robotToCamera, stdDevFactor);
        }

        public String getName() {
            return data.name();
        }

        public Transform3d getRobotToCamera() {
            return data.robotToCamera();
        }

        public double getStdDevFactor() {
            return data.stdDevFactor();
        }

        public CameraData getData() {
            return data;
        }
    }
}
