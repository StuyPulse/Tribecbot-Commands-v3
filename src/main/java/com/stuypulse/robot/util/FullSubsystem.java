/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import java.util.ArrayList;
import java.util.List;

/**
 * A standard subsystem that includes an extra periodic callback which runs after the command
 * scheduler. Allows outputs to be published after all other periodic code has finished.
 */
public abstract class FullSubsystem extends Mechanism {
  private static List<FullSubsystem> instances = new ArrayList<>();

  public FullSubsystem() {
    super();
    instances.add(this);
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public FullSubsystem(String name) {
    super(name);
    instances.add(this);
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  /**
   * This method is called periodically before scheduled commands, and should be overriden for
   * updating inputs.
   */
  public void periodic() {}
  ;

  /**
   * This method is called periodically after the command scheduler, and should be overriden for
   * applying outputs.
   */
  public void periodicAfterScheduler() {}
  ;

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
