package com.stuypulse.robot.subsystems.handoff;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

public interface HandoffIO {
  @AutoLog
  public static class HandoffIOInputs {
    public Current motorLeadSupplyCurrent = Amps.zero();
    public Current motorLeadStatorCurrent = Amps.zero();
    public Temperature motorLeadTemperature = Celsius.zero();
    public AngularVelocity motorLeadVelocity = DegreesPerSecond.zero();
    public Voltage motorLeadAppliedVoltage = Volts.zero();

    public Current motorFollowSupplyCurrent = Amps.zero();
    public Current motorFollowStatorCurrent = Amps.zero();
    public Temperature motorFollowTemperature = Celsius.zero();
    public AngularVelocity motorFollowVelocity = DegreesPerSecond.zero();
    public Voltage motorFollowAppliedVoltage = Volts.zero();
  }

  public static class HandoffIOOutputs {
    public double handoffDutyCycle = 0.0;
  }

  public default void updateInputs(HandoffIOInputs inputs) {}

  public default void applyOutputs(HandoffIOOutputs outputs) {}
}
