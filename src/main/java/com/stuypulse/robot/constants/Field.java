/** ********************** PROJECT TRIBECBOT ************************ */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.constants;

import static org.wpilib.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.vision.apriltag.AprilTag;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.vision.Apriltag;

/**
 * This interface stores information about the field elements.
 */
public interface Field {

    public static final Field2d FIELD2D = new Field2d();

    public static Distance WIDTH = Inches.of(317.000);
    public static Distance LENGTH = Inches.of(651.200);

    public static final Distance TRENCH_HOOD_TOLERANCE = Inches.of(20);

    // Alliance relative hub center coordinates
    public static final Pose2d HUB_CENTER =
      new Pose2d(Inches.of(182.11), WIDTH.div(2), new Rotation2d());
    public static final Pose2d HUB_FAR_RIGHT_CORNER =
      new Pose2d(Inches.of(205.6), WIDTH.div(2).plus(Inches.of(47 / 2.0)), Rotation2d.kZero);
    public static final Pose2d HUB_FAR_LEFT_CORNER =
      new Pose2d(Inches.of(205.6), WIDTH.div(2).plus(Inches.of(47 / 2.0)), Rotation2d.kZero);

    public static final Distance HUB_RADIUS = Inches.of(41.7 / 2.0);

    public static final Distance OPPONENT_ZONE_X = LENGTH.minus(Inches.of(158.6));

    public static final Distance OPPONENT_HUB_DS_X = LENGTH.minus(HUB_FAR_LEFT_CORNER.getMeasureX()).plus(HUB_RADIUS.times(2.0));

    public static final Distance BEHIND_HUB_TOLERANCE_X = Inches.of(144); // To extend the triangle vertex
    public static final Distance BEHIND_HUB_TOLERANCE_Y = Inches.of(12).plus(Inches.of(2)); // To extend
                                                                                                            // base of
                                                                                                            // triangle
                                                                                                            // (colinear
                                                                                                            // with back
                                                                                                            // hub)

    public static final Pose2d BEHIND_HUB_TRIANGLE_VERTEX = new Pose2d(
            Inches.of(182.11).plus(BEHIND_HUB_TOLERANCE_X), WIDTH.div(2.0), new Rotation2d());

    public static Pose2d getHubPose() {
        return HUB_CENTER;
    }

    // Alliance relative tower center coordinates
    public final Pose2d TOWER_FAR_CENTER = new Pose2d(Units.inchesToMeters(42.0), Units.inchesToMeters(147.47),
            new Rotation2d());
    public final Pose2d TOWER_FAR_RIGHT = new Pose2d(Units.inchesToMeters(42.0),
            Units.inchesToMeters(147.47 - 23.5 - 10), new Rotation2d());
    public final Pose2d TOWER_FAR_LEFT = new Pose2d(Units.inchesToMeters(42.0),
            Units.inchesToMeters(147.47 + 23.5 - 5 + 3.5), new Rotation2d());
    public final double TOWER_BAR_DISPLACEMENT = Units.inchesToMeters(11.38);

    public final double DISTANCE_TO_RUNGS = Units.inchesToMeters(20); // placeholder value, how far away in terms of
                                                                      // y-cord from the rung

    public static boolean closerToTop() {
        return CommandSwerveDrivetrain.getInstance().getPose().getY() >= Field.TOWER_FAR_CENTER.getY();
    }

    public final Pose2d INNER_LEFT_FERRY_ZONE = new Pose2d(
            Inches.of(31.5),
            WIDTH.minus(Inches.of(34.5)).minus(Inches.of(48)),
            new Rotation2d());

    public final Pose2d INNER_RIGHT_FERRY_ZONE = new Pose2d(
            Inches.of(20.75),
            Inches.of(76).plus(Inches.of(48)),
            new Rotation2d());

    public final Pose2d OUTER_LEFT_FERRY_ZONE = new Pose2d(
            Inches.of(31.5),
            WIDTH.minus(Inches.of(34.5)),
            new Rotation2d());

    public final Pose2d OUTER_RIGHT_FERRY_ZONE = new Pose2d(
            Units.inchesToMeters(20.75),
            Units.inchesToMeters(76),
            new Rotation2d());

    public final Distance FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE = Inches.of(75);

    public static Pose2d getFerryZonePose(Translation2d robot) {
        Distance fieldMidY = WIDTH.div(2);

        if (robot.getMeasureY().gt(fieldMidY)) {
            if (robot.getMeasureY().gt(WIDTH.minus(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE))) {
                return INNER_LEFT_FERRY_ZONE;
            } else {
                return OUTER_LEFT_FERRY_ZONE;
            }
        } else {
            if (robot.getMeasureY().lt(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE)) {
                return INNER_RIGHT_FERRY_ZONE;
            } else {
                return OUTER_RIGHT_FERRY_ZONE;
            }
        }
    }

    /**
     * * TRENCH COORDINATES **
     */
    public interface AllianceLeftTrench {

        public static final Pose2d leftEdge = new Pose2d(Inches.of(182.11), WIDTH, new Rotation2d());
        public static final Pose2d rightEdge = new Pose2d(Inches.of(182.11),
                WIDTH.minus(Inches.of(50.59)), new Rotation2d());
    }

    public interface AllianceRightTrench {

        public static final Pose2d leftEdge = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(50.59),
                new Rotation2d());
        public static final Pose2d rightEdge = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(0),
                new Rotation2d());
    }

    // OPPONENT SIDE, BUT LEFT/RIGHT RELATIVE TO YOUR ALLIANCE POV
    public interface OpponentLeftTrench {

        public static final Pose2d leftEdge = new Pose2d(LENGTH.minus(Inches.of(182.11)), WIDTH,
                new Rotation2d());
        public static final Pose2d rightEdge = new Pose2d(LENGTH.minus(Inches.of(182.11)),
                WIDTH.minus(Inches.of(50.59)), new Rotation2d());
    }

    // OPPONENT SIDE, BUT LEFT/RIGHT RELATIVE TO YOUR ALLIANCE POV
    public interface OpponentRightTrench {

        public static final Pose2d leftEdge = new Pose2d(LENGTH.minus(Inches.of(182.11)),
                Inches.of(50.59), new Rotation2d());
        public static final Pose2d rightEdge = new Pose2d(LENGTH.minus(Inches.of(182.11)),
                Inches.of(0), new Rotation2d());
    }

    /**
     * * APRILTAGS **
     */
    public static enum NamedTags {
        RED_RIGHT_TRENCH_NZ, // #1
        RED_HUB_RIGHT_SIDE_MID,
        RED_HUB_BACK_SIDE_LEFT,
        RED_HUB_BACK_SIDE_MID,
        RED_HUB_LEFT_SIDE_MID, // #5
        RED_LEFT_TRENCH_NZ,
        RED_LEFT_TRENCH_AZ,
        RED_HUB_LEFT_SIDE_RIGHT,
        RED_HUB_FRONT_SIDE_LEFT,
        RED_HUB_FRONT_SIDE_MID, // #10
        RED_HUB_RIGHT_SIDE_LEFT,
        RED_RIGHT_TRENCH_AZ,
        RED_HP_MID,
        RED_HP_RIGHT,
        RED_TOWER_MID, // #15
        RED_TOWER_RIGHT,
        BLUE_RIGHT_TRENCH_NZ,
        BLUE_HUB_RIGHT_SIDE_MID,
        BLUE_HUB_BACK_SIDE_LEFT,
        BLUE_HUB_BACK_SIDE_MID, // #20
        BLUE_HUB_LEFT_SIDE_MID,
        BLUE_LEFT_TRENCH_NZ,
        BLUE_LEFT_TRENCH_AZ,
        BLUE_HUB_LEFT_SIDE_RIGHT,
        BLUE_HUB_FRONT_SIDE_LEFT, // #25
        BLUE_HUB_FRONT_SIDE_MID,
        BLUE_HUB_RIGHT_SIDE_LEFT,
        BLUE_RIGHT_TRENCH_AZ,
        BLUE_HP_MID,
        BLUE_HP_RIGHT, // #30
        BLUE_TOWER_MID,
        BLUE_TOWER_RIGHT;

        public final Apriltag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return Robot.isBlue()
                    ? tag.getLocation()
                    : transformToOppositeAlliance(tag.getLocation());
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    Apriltag APRILTAGS[] = {
            // 2026 Field AprilTag Layout
            new Apriltag(1, new Pose3d(
                    new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(292.31),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(2, new Pose3d(
                    new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(182.60),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
            new Apriltag(3, new Pose3d(
                    new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(172.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(4, new Pose3d(
                    new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(158.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(5, new Pose3d(
                    new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(135.09),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
            new Apriltag(6, new Pose3d(
                    new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(25.37),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(7, new Pose3d(
                    new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(25.37),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(8, new Pose3d(
                    new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(135.09),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
            new Apriltag(9, new Pose3d(
                    new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(144.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(10, new Pose3d(
                    new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(158.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(11, new Pose3d(
                    new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(182.60),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
            new Apriltag(12, new Pose3d(
                    new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(292.31),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(13, new Pose3d(
                    new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(291.47),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(14, new Pose3d(
                    new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(274.47),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(15, new Pose3d(
                    new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(170.22),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(16, new Pose3d(
                    new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(153.22),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(17, new Pose3d(
                    new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(25.37),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(18, new Pose3d(
                    new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(135.09),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
            new Apriltag(19, new Pose3d(
                    new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(144.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(20, new Pose3d(
                    new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(158.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(21, new Pose3d(
                    new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(182.60),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
            new Apriltag(22, new Pose3d(
                    new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(292.31),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(23, new Pose3d(
                    new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(292.31),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(24, new Pose3d(
                    new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(182.60),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
            new Apriltag(25, new Pose3d(
                    new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(172.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(26, new Pose3d(
                    new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(158.84),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(27, new Pose3d(
                    new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(135.09),
                            Units.inchesToMeters(44.25)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
            new Apriltag(28, new Pose3d(
                    new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(25.37),
                            Units.inchesToMeters(35.00)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
            new Apriltag(29, new Pose3d(
                    new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(26.22),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(30, new Pose3d(
                    new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(43.22),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(31, new Pose3d(
                    new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(147.47),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
            new Apriltag(32, new Pose3d(
                    new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(164.47),
                            Units.inchesToMeters(21.75)),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))))
    };

    public static boolean isValidTag(int id) {
        for (Apriltag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static Apriltag getTag(int id) {
        for (Apriltag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

    public final int[] RED_HUB_TAG_IDS = { 2, 3, 4, 5, 8, 9, 10, 11 };
    public final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };
    public final int[] RED_TRENCH_TAG_IDS = { 1, 6, 7, 12 };
    public final int[] BLUE_TRENCH_TAG_IDS = { 17, 22, 23, 28 };
    public final int[] RED_TOWER_TAG_IDS = { 15, 16 };
    public final int[] BLUE_TOWER_TAG_IDS = { 31, 32 };
    public final int[] RED_OUTPOST_TAG_IDS = { 13, 14 };
    public final int[] BLUE_OUTPOST_TAG_IDS = { 29, 30 };
    public final int[] ALL_TAGS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
            24, 25, 26, 27, 28, 29, 30, 31, 32 };

    /* TRANSFORM FUNCTIONS */
    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
                rotated.getTranslation().plus(new Translation3d(LENGTH.magnitude(), WIDTH.magnitude(), 0)),
                rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
                rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
                rotated.getRotation());
    }

    public static Translation2d transformToOppositeAlliance(Translation2d translation) {
        return new Translation2d(LENGTH.minus(translation.getMeasureX()), WIDTH.minus(translation.getMeasureY()));
    }

    public static List<Pose2d> transformToOppositeAlliance(List<Pose2d> poses) {
        List<Pose2d> newPoses = new ArrayList<>();
        for (Pose2d pose : poses) {
            newPoses.add(transformToOppositeAlliance(pose));
        }
        return newPoses;
    }

    /**
     * ** EMPTY FIELD POSES ***
     */
    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject) {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}
