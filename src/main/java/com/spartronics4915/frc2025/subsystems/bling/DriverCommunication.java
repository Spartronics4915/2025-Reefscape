package com.spartronics4915.frc2025.subsystems.bling;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class DriverCommunication extends BlingSegment {
    public enum Message {
        OFF(LEDPattern.solid(Color.kBlack)),
        WARNING(LEDPattern.solid(Color.kYellow)),
        GOOD(LEDPattern.solid(Color.kGreen)),
        UH_OH(LEDPattern.solid(Color.kRed));

        private final LEDPattern pattern;

        private Message(LEDPattern pattern) {
            this.pattern = pattern;
        }
    }

    private Message message = Message.OFF;

    public DriverCommunication(int length) {
        this.ledLength = length;
    }

    public Message getCurrentMessage() {
        return message;
    }

    public void setCurrentMessage(Message message) {
        this.message = message;
    }

    @Override
    protected void updateLights() {
        // Periodic logic goes here
        message.pattern.applyTo(buffer);
    }

}
