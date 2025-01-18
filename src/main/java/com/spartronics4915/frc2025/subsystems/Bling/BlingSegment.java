package com.spartronics4915.frc2025.subsystems.Bling;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public abstract class BlingSegment {
    protected int frame = 0;
    protected int ledLength;
    protected int maxLength = -1;
    AddressableLEDBufferView buffer;

    final public int getStripLength() {
        return this.ledLength;
    }

    final private void incrementFrame() {
        frame++;
        if (maxLength > -1)
            if (frame > maxLength) frame = 0;
    }

    final public void update() {
        updateLights();
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

}
