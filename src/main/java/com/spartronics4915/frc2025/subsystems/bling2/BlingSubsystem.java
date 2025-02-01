package com.spartronics4915.frc2025.subsystems.bling2;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlingSubsystem extends SubsystemBase {
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private BlingSegment[] segments;

    public void updateSegments(BlingSegment... segments) {
        this.segments = segments;

        int ledLength = 0;
        for (BlingSegment x : segments) {
            ledLength += x.getStripLength();
        }

        strip.setLength(ledLength);
        buffer = new AddressableLEDBuffer(ledLength);

        int index = 0;
        for (BlingSegment x : segments) {
            x.buffer = buffer.createView(index, index + x.ledLength - 1);
            index += x.ledLength;
        }
        
        strip.start();
    }

    public BlingSubsystem(int port, BlingSegment... shows) {
        strip = new AddressableLED(port);
        updateSegments(shows);
    }

    @Override
    public void periodic() {
        for (BlingSegment show : segments) {
            show.update();
        }
        strip.setData(buffer);
    }

}
