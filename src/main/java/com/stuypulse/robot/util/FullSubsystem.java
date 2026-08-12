// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

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
   * This method is called periodically before scheduled commands, and should be overriden 
   * for updating inputs.
   */
  public void periodic() {};

  /**
   * This method is called periodically after the command scheduler, and should be overriden 
   * for applying outputs.
   */
  public void periodicAfterScheduler() {};

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}