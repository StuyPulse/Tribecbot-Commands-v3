/************************ PROJECT SSSS ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.*;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * File containing tunable settings for every subsystem on the robot.
 *
 * <p>We use DogLog's tunables in order to have tunable values that we can edit from external
 * dashboards.
 */
public interface Settings {
  public static final Time DT = Milliseconds.of(20);
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum VisionMode {
    LIMELIGHT,
    PHOTON
  }

  public static final VisionMode currentVisionMode = VisionMode.PHOTON;

  public interface EnabledSubsystems {
    LoggedNetworkBoolean INTAKE = new LoggedNetworkBoolean("Enabled Subsystems/Intake", true);
    LoggedNetworkBoolean HANDOFF = new LoggedNetworkBoolean("Enabled Subsystems/Handoff", true);
    LoggedNetworkBoolean HOOD = new LoggedNetworkBoolean("Enabled Subsystems/Hood", true);
    LoggedNetworkBoolean SHOOTER = new LoggedNetworkBoolean("Enabled Subsystems/Shooter", true);
    LoggedNetworkBoolean TURRET = new LoggedNetworkBoolean("Enabled Subsystems/Turret", true);
    LoggedNetworkBoolean SPINDEXER = new LoggedNetworkBoolean("Enabled Subsystems/Spindexer", true);
    LoggedNetworkBoolean SWERVE = new LoggedNetworkBoolean("Enabled Subsystems/Swerve", true);
  }

  public interface Superstructure {
    public final double SHOOTER_TOLERANCE_RPM_HIGH = 50.0;
    public final double SHOOTER_TOLERANCE_RPM_LOW = 80.0;
    public final double SHOOTER_SOTM_TOLERANCE_RPM_HIGH = 100.0;
    public final double SHOOTER_SOTM_TOLERANCE_RPM_LOW = 100.0;
    public final double SHOOTER_FOTM_TOLERANCE_RPM_HIGH = 150.0;
    public final double SHOOTER_FOTM_TOLERANCE_RPM_LOW = 250.0;

    public final double IS_EMPTY_RPM_TOLERANCE = 150; // TODO: update IS EMPTY VALUE
    public final double IS_EMPTY_DEBOUNCE_TIME = 0.4; // TODO: update IS EMPTY VALUE

    public final Angle HOOD_TOLERANCE = Degrees.of(0.5);
    public final Angle HOOD_SOTM_TOLERANCE = Degrees.of(2);

    public interface AngleInterpolation {
      double[][] distanceAngleInterpolationValues = {
        {0.96, Units.degreesToRadians(15)},
        {1.22, Units.degreesToRadians(20)},
        {2.15, Units.degreesToRadians(27)},
        {3.38, Units.degreesToRadians(34)},
        {4.43, Units.degreesToRadians(39)},
        {5.66, Units.degreesToRadians(39)},
        {6.44, Units.degreesToRadians(44)}
      };
    }

    public interface RPMInterpolation {
      double[][] distanceRPMInterpolationValues = {
        {0.96, 2800},
        {1.22, 2600.0},
        {2.15, 2805.0},
        {3.38, 3075},
        {4.43, 3350.0},
        {5.66, 3650.0},
        {6.44, 3800.0},
        {8.23, 4500.0} // THIS POINT IS AN EXTRAPOLATION
      };
    }

    public interface TOFInterpolation {
      double[][] distanceTOFInterpolationValues = {
        {0.96, 1.055},
        {1.22, 0.965}, // seconds
        {2.15, 1.01},
        {3.38, 1.02},
        {4.43, 1.165},
        {5.50, 1.21},
        {6.44, 1.255},
        {6.6, 1.41},
        {8.23, 1.71} // THIS POINT IS AN EXTRAPOLATION
      };
    }

    public interface FerryRPMInterpolation {
      double[][] ferryDistanceRPMInterpolation = {
        // Lab
        {1, 2000},
        {5.16, 3300.0},
        {6.94, 3600.0},
        {7.87, 3800.0},
        {9.77, 4300.0}, // TODO: ADD DATA BACK IN COMP
        {10.694, 4700.0}, // STARTING FROM HERE THE DATA IS EXTRAPOLATED!!!
        {11.516, 4900.0}
      };
    }

    public interface FerryTOFInterpolation {
      double[][] FerryTOFInterpolationInterpolation = {
        {5.16, 1.16},
        {6.94, 1.37},
        {7.87, 1.57},
        {9.77, 1.64},
        {10.694, 1.765}, // extrapolated
        {11.516, 1.838}, // extrapolated
        {12.416, 1.914}, // extrapolated
        {13.316, 1.988}, // extrapolated
        {14.216, 2.060}, // extrapolated
        {15.148, 2.131}, // extrapolated
        {16.54, 2.234}, // extrapolated (field length)
      };
    }

    public interface Shooter {

      public final double IS_SHOOTING_CURRENT = 25.0;

      public final double GEAR_RATIO = 1.0;
      public final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.965 / 2.0);

      public interface RPM {
        public final LoggedNetworkNumber MANUAL_OVERRIDE =
            new LoggedNetworkNumber("InterpolationTesting/Shoot State Target RPM", 3863.0);

        public final AngularVelocity REVERSE = RPM.zero();
        public final AngularVelocity KB = RPM.of(2675.0);
        public final AngularVelocity LEFT_CORNER = RPM.of(3650.0);
        public final AngularVelocity RIGHT_CORNER = RPM.of(3650.0);
      }
    }

    public interface Hood {
      /**
       * DISCLAIMER: THERE IS NO ABS ENCODER ON THE BOT RN The absolute encoder is mounted on a 11:1
       * gear reduction relative to the hood mechanism. This means:
       *
       * <p>- The encoder rotates 11 times for every 1 full rotation of the hood. - The hood's
       * physical range of motion is only 30 degrees.
       *
       * <p>Because 30° * 11 = 330°, the encoder will never exceed 360° over the entire hood travel.
       * Therefore, the absolute encoder reading (0–330°) uniquely maps to the hood’s 0–30°
       * mechanical range without any ambiguity.
       */
      public final double GEAR_RATIO = 125.4;

      public final double ENCODER_TO_MECH = 11.0;
      public final Voltage HOOD_HOMING_VOLTAGE = Volts.of(0.5);

      public final Angle ENCODER_OFFSET = Rotations.of(0.795);

      public final Angle MAX_FROM_HORIZON = Degrees.of(45.0);
      public final Angle MIN_FROM_HORIZON = Degrees.of(15.0);
      public final Angle SOFT_LIMIT = Degrees.of(.25);
      public final Angle FORWARD_SOFT_LIMIT = MAX_FROM_HORIZON.minus(SOFT_LIMIT);
      public final Angle REVERSE_SOFT_LIMIT = MIN_FROM_HORIZON.plus(SOFT_LIMIT);

      public final Current STALL_CURRENT_LIMIT = Amps.of(0.55);
      public final double STALL_DEBOUNCE = 0.5;

      public interface Angles {
        public final LoggedNetworkNumber MANUAL_OVERRIDE =
            new LoggedNetworkNumber("InterpolationTesting/Shoot State Target Angle (deg)", 44.0);
        public final Angle MAX = FORWARD_SOFT_LIMIT;
        public final Angle MIN = REVERSE_SOFT_LIMIT;
        public final Angle FERRY_ANGLE = MAX; // Degrees.of(44.0);

        public final Angle STOW = Degrees.of(21.0);
        public final Angle KB = Degrees.of(20.0);
        public final Angle LEFT_CORNER = Degrees.of(39.0);
        public final Angle RIGHT_CORNER = Degrees.of(39.0);
      }
    }

    public interface Turret {
      // public final AngularVelocity MAX_VEL = new
      // Angle(Units.degreesToRadians(600.0));
      // public final Angle MAX_ACCEL = new Angle(Units.degreesToRadians(600.0));
      public final Angle TOLERANCE = Degrees.of(2.0);
      public final LoggedNetworkNumber SOTM_TOLERANCE_THRESHOLD_METERS =
          new LoggedNetworkNumber(
              "Superstructure/Turret/SOTM Tolerance Dist Threshold (Meters)", 1.75);
      public final LoggedNetworkNumber SOTM_TOLERANCE_CLOSE =
          new LoggedNetworkNumber("Superstructure/Turret/SOTM Tolerance (Close)", 10.0);
      public final LoggedNetworkNumber SOTM_TOLERANCE_FAR =
          new LoggedNetworkNumber(
              "Superstructure/Turret/SOTM Tolerance (Far)", 6.0); // Degrees.of(10.0);
      public final Angle FOTM_TOLERANCE = Degrees.of(10.0);

      public final Angle KB = Degrees.of(0.0);
      public final Angle LEFT_CORNER = Degrees.of(-233.0);
      public final Angle RIGHT_CORNER = Degrees.of(53.0);

      double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
      double WRAP_DEBOUNCE = 0.5;
      double SETPOINT_FILTER_THRESHOLD_DEG = 0.5;

      Angle MAX_THEORETICAL_ROTATION = Degrees.of(612);
      Angle MIN_THEORETICAL_ROTATION = Degrees.of(-612);

      /* CONSTANTS */
      public final double RANGE_CW = 90.0; // -360.0;
      public final double RANGE_CCW = -360.0; // 85.0; // -397.0 is further

      public final Angle GAIN_SWITCHING_THRESHOLD_START = Degrees.of(30);
      public final Angle GAIN_SWITCHING_THRESHOLD_END = Degrees.of(3);

      public final Translation2d TURRET_OFFSET =
          new Translation2d(Units.inchesToMeters(-4.0), Units.inchesToMeters(8.0));
      public final double TURRET_HEIGHT = Units.inchesToMeters(0.0);

      public final double GEAR_RATIO_MOTOR_TO_MECH = (60.0 / 9.0) * (95.0 / 12.0); // 1425.0 / 36.0;

      // public final LoggedNetworkNumber ARBITRARY_kA_TERM = new
      // LoggedNetworkNumber("Superstructure/Turret/Gains/arbitrary kA", 1.5);

      public interface BigGear {
        public final int TEETH = 95;
      }

      public interface Encoder17t {
        public final int TEETH = 17;
        public final Angle OFFSET = Rotations.of(-0.185);
      }

      public interface Encoder18t {
        public final int TEETH = 18;
        public final Angle OFFSET = Rotations.of(-0.814);
      }

      public interface SoftwareLimit {
        public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
        public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
      }
    }

    public interface SOTM {
      public final int MAX_ITERATIONS = 10;
      double TIME_TOLERANCE = 1e-3;
      LoggedNetworkNumber UPDATE_DELAY =
          new LoggedNetworkNumber("Superstructure/SOTM/update delay", 0.05);
    }
  }
}
