package com.spartronics4915.frc2025.subsystems.Bling;

import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class DriverCommunication extends BlingSegment {
    LimelightVisionSubsystem vision;

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

    public DriverCommunication(int length, VisionDeviceSubystem vision) {
        this.ledLength = length;
        this.vision = (LimelightVisionSubsystem) vision;
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
        if (vision.canSeeTags())
            Message.GOOD.pattern.applyTo(buffer);
        else
            Message.UH_OH.pattern.applyTo(buffer);
    }

}
