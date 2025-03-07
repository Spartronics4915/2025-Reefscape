package com.spartronics4915.frc2025.subsystems.bling2;

import static com.spartronics4915.frc2025.Constants.BlingConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlingSubsystem extends SubsystemBase {
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private BlingSegment[] segments;
    private String[] hexStrings;
    private int blingPort;

    public void updateSegments(BlingSegment... segments) {
        this.segments = segments;

        int ledLength = 0;
        for (BlingSegment x : segments) {
            ledLength += x.getStripLength();
        }

        if (LIGHTS_ENABLED) strip.setLength(ledLength);
        buffer = new AddressableLEDBuffer(ledLength);

        hexStrings = new String[ledLength];

        int index = 0;
        for (BlingSegment x : segments) {
            x.buffer = buffer.createView(index, index + x.ledLength - 1);
            index += x.ledLength;
        }
        
        if (LIGHTS_ENABLED) strip.start();
    }

    public BlingSubsystem(int port, BlingSegment... shows) {
        this.blingPort = port;
        if (LIGHTS_ENABLED) strip = new AddressableLED(port);
        updateSegments(shows);

        SmartDashboard.putData("Stop lights", clearLights());
    }

    private void logLEDs() {
        for(int i = 0; i < hexStrings.length; i++) {
            int brightnessMultiplier = (100/BLING_BRIGHTNESS);
            Color ledColor = new Color(buffer.getRed(i) * brightnessMultiplier, buffer.getGreen(i) * brightnessMultiplier,buffer.getBlue(i) * brightnessMultiplier);
            hexStrings[i] = ledColor.toHexString();
        }
        SmartDashboard.putStringArray("Bling", hexStrings);
    }

    /**
     * Clears the memory of the lights, only works if lights are disabled
     */
    public Command clearLights() {
        return Commands.runOnce(() -> {
            if (LIGHTS_ENABLED) {
                strip.stop();
                strip.close();
                LIGHTS_ENABLED = false;
            } else {
                System.out.println("worked");
                AddressableLED clearStrip = new AddressableLED(blingPort);
                clearStrip.stop();
                clearStrip.close();
            }
        });
    }

    @Override
    public void periodic() {
        for (BlingSegment show : segments) {
            show.update();
        }
        if (LIGHTS_ENABLED) strip.setData(buffer);
        logLEDs();
    }

}
