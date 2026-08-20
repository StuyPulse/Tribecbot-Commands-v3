package com.stuypulse.robot.subsystems.leds;

import org.wpilib.simulation.AddressableLEDSim;

public class LEDIOSim implements LEDIO {
    private final AddressableLEDSim ledSim;
    private final byte[] ledData = new byte[4];

    private final byte red = 0;
    private final byte green = 0;
    private final byte blue = 0;
    private final byte white = 0;

    public boolean isInitialized = false;

    public LEDIOSim() {
        ledSim = new AddressableLEDSim(LEDConstants.Ports.CANDLE_PORT);

        ledSim.setInitialized(isInitialized);
        ledSim.setData(new byte[]{red, green, blue, white});
        ledSim.setLength(LEDConstants.Settings.LED_LENGTH);
        ledSim.setStart(0);
    }

    
@Override
public void updateInputs(LEDIOInputs inputs) {
    inputs.isConnected = ledSim.getInitialized();
    }

public void setOutputs(LEDIOOutputs outputs) {
    ledSim.setData(new byte[]{(byte) outputs.color.Red, 
        (byte) outputs.color.Green, 
        (byte) outputs.color.Blue, 
        (byte) outputs.color.White});
    }
}
