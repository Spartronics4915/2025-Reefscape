package com.spartronics4915.frc2025.subsystems.bling2;

import edu.wpi.first.wpilibj.LEDPattern;

public class BlingLEDPattern extends BlingSegment {
    private LEDPattern pattern;

    public BlingLEDPattern(LEDPattern pattern, int length) {
        this.pattern = pattern;
        this.ledLength = length;
    }

    @Override
    protected void updateLights() {
        pattern.applyTo(buffer);
    }

}
