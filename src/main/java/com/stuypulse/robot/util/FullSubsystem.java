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
import java.util.function.BooleanSupplier;

/**
 * A standard subsystem that includes an extra periodic callback which runs after the command
 * scheduler. Allows outputs to be published after all other periodic code has finished.
 */
public abstract class FullSubsystem extends Mechanism {
  private static List<FullSubsystem> instances = new ArrayList<>();
  private BooleanSupplier isEnabled;

  public FullSubsystem(String name, BooleanSupplier enabledSupplier) {
    super(name);
    this.isEnabled = enabledSupplier;
    instances.add(this);
    Scheduler.getDefault().addPeriodic(this::periodicLoop);
  }

  public FullSubsystem(String name) {
    this(name, () -> true);
  }

  public FullSubsystem() {
    super();
    this.isEnabled = () -> true;
    instances.add(this);
    Scheduler.getDefault().addPeriodic(this::periodicLoop);
  }

  public FullSubsystem(BooleanSupplier enabledSupplier) {
    super();
    this.isEnabled = enabledSupplier;
    instances.add(this);
    Scheduler.getDefault().addPeriodic(this::periodicLoop);
  }

  /**
   * This method is called periodically before scheduled commands, and should be overriden for
   * updating inputs. This runs regardless of if the subsystem is enabled or disabled.
   */
  public void periodic() {}

  /**
   * This method is called periodically only if the subsystem is disabled, and should be overriden
   * for actual subsystem logic
   */
  public void enabledPeriodic() {}

  /**
   * This method is called periodically after the command scheduler, and should be overriden for
   * applying outputs.
   */
  public void periodicAfterScheduler() {}

  private void periodicLoop() {
    periodic();

    if (!isEnabled.getAsBoolean()) {
      disabledPeriodic();
      return;
    }

    enabledPeriodic();
  }

  /**
   * This method is called periodically when the subsystem is disabled, and should be overriden
   * for stopping motors and zeroing out values.
   */
  public void disabledPeriodic() {}

  public boolean isEnabled() {
    return this.isEnabled.getAsBoolean();
  }

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
