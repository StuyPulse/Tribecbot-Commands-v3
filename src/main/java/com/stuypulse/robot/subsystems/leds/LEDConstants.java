/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.leds;

import org.wpilib.math.util.Units;
import org.wpilib.util.Color;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

public final class LEDConstants {
  private LEDConstants() {}

  public static interface Settings {
    public SolidColor solidColorRequest =
        new SolidColor(0, LEDConstants.Settings.LED_LENGTH - 1).withColor(new RGBWColor(Color.RED));
    public RainbowAnimation rainbowRequest =
        new RainbowAnimation(0, LEDConstants.Settings.LED_LENGTH - 1).withFrameRate(60).withSlot(0);

    public static RGBWColor rgbwConverter(Color color) {
      return new RGBWColor(color);
    }

    public final int LED_LENGTH = 8 + 21; // CANdle already has 8
    RGBWColor PASSING_TRENCH = rgbwConverter(Color.RED);
    RGBWColor IS_BEHIND_HUB = rgbwConverter(Color.RED);

    // RGBWColor CLIMB_ALIGNING = rgbwConverter(Color.YELLOW);
    // RGBWColor CLIMB_ALIGNED = rgbwConverter(Color.GREEN);
    // RGBWColor CLIMBING = rgbwConverter(Color.RED);

    RGBWColor TURRET_WRAPPING = rgbwConverter(Color.RED);
    // RGBWColor LEFT_WARNING = rgbwConverter(Color.BLACK); // TBD
    // RGBWColor RIGHT_WARNING = rgbwConverter(Color.BLACK); // TBD

    RGBWColor SHOOT_IN_PLACE = rgbwConverter(Color.PURPLE);

    RGBWColor SOTM_ON = rgbwConverter(Color.GREEN);
    RGBWColor FOTM_ON = rgbwConverter(Color.DARK_BLUE);
    RGBWColor LEFT_CORNER = rgbwConverter(Color.PURPLE);
    RGBWColor RIGHT_CORNER = rgbwConverter(Color.BLUE);

    RGBWColor KB_DISTANCE = rgbwConverter(Color.PINK);

    // RGBWColor REVERSE = rgbwConverter(Color.WHITE);
    RGBWColor STOP_ROLLERS = rgbwConverter(Color.YELLOW);

    RGBWColor RESET_HEADING = rgbwConverter(Color.YELLOW);
    RGBWColor X_WHEELS = rgbwConverter(Color.RED);

    RGBWColor INTAKE_STOW = rgbwConverter(Color.BROWN); // broken
    RGBWColor INTAKE_DEPLOYED = rgbwConverter(Color.PURPLE); // broken

    RGBWColor DISABLED_ALIGNED = rgbwConverter(Color.GREEN);
    RGBWColor DISABLED = rgbwConverter(Color.RED);

    RGBWColor AUTON_ONE = rgbwConverter(Color.BLUE);
    RGBWColor AUTON_TWO = rgbwConverter(Color.ORANGE);

    RGBWColor LLDEAD = rgbwConverter(Color.WHITE);

    SolidColor RIGHT_DEAD_STRIP =
        new SolidColor(LEDConstants.Settings.LED_LENGTH - 6, LEDConstants.Settings.LED_LENGTH - 2);
    SolidColor BACK_DEAD_STRIP =
        new SolidColor(LEDConstants.Settings.LED_LENGTH - 13, LEDConstants.Settings.LED_LENGTH - 9);
    SolidColor LEFT_DEAD_STRIP =
        new SolidColor(
            LEDConstants.Settings.LED_LENGTH - 20, LEDConstants.Settings.LED_LENGTH - 16);
    SolidColor CANDLE_DEAD_STRIP = new SolidColor(0, 7);

    // RGBWColor.gradient(GradientType.kDiscontinuous, Color.kRed,
    // Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));

    public final int DESIRED_TAGS_WHEN_DISABLED = 2;

    public double APRIL_TAG_DISTANCE_THRESHOLD =
        Units.feetToMeters(
            2); // TODO: update because comparing Translation2d, so make sure it is 2 feet
  }

  public interface Ports {
    int LED_PORT = 1;
    int CANDLE_PORT = 61;
  }
}
