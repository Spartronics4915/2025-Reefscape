package com.spartronics4915.frc2025.subsystems.bling2;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2025.Constants.BlingConstants;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public abstract class BlingSegment {
    protected int realFrame = 0;
    protected int frame = 0;
    protected int ledLength;
    protected int maxLength = -1;
    AddressableLEDBufferView buffer;

    final public int getStripLength() {
        return this.ledLength;
    }

    final protected void incrementFrame(int amount) {
        realFrame += amount;
        frame = realFrame / BlingConstants.FRAME_WAIT;
        if (maxLength > -1)
            if (frame >= maxLength) realFrame = 1;
    }
    protected final void incrementFrame() {
        this.incrementFrame(1);
    }

    final protected void update() {
        if (realFrame % BlingConstants.FRAME_WAIT == 0) {
            updateLights();
        }
        incrementFrame();
    }
    abstract protected void updateLights();

    public static final BlingShow show(String filename) {
        return new BlingShow(filename);
    }
    public static final BlingLEDPattern solid(int r, int g, int b, int length) {
        return new BlingLEDPattern(LEDPattern.solid(new Color(r,g,b)), length);
    }
    public static final BlingLEDPattern solid(Color color, int length) {
        return new BlingLEDPattern(LEDPattern.solid(color), length);
    }
    public static final BlingLEDPattern rainbow(int length) {
        return new BlingLEDPattern(LEDPattern.rainbow(255,255), length);
    }
    public static final BlingLEDPattern scrollingRainbow(int length, double speed) {
        return new BlingLEDPattern(LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), Meters.of(1)), length);
    }
    public static final BlingLEDPattern pulseColor(int length, Color color, double pulseLength) {
        return new BlingLEDPattern(LEDPattern.solid(color).breathe(Seconds.of(pulseLength)), length);
    }

}
