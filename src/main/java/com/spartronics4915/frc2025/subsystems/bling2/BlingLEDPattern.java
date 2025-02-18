package com.spartronics4915.frc2025.subsystems.bling2;

import static edu.wpi.first.units.Units.Percent;

import com.spartronics4915.frc2025.Constants.BlingConstants;

import edu.wpi.first.wpilibj.LEDPattern;

public class BlingLEDPattern extends BlingSegment {
    private LEDPattern pattern;

    public BlingLEDPattern(LEDPattern pattern, int length) {
        this.pattern = pattern.atBrightness(Percent.of(BlingConstants.BLING_BRIGHTNESS));
        this.ledLength = length;
    }

    @Override
    protected void updateLights() {
        pattern.applyTo(buffer);
    }

}
