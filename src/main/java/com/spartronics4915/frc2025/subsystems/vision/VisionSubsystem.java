package com.spartronics4915.frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private final LimelightDevice limelight;

    private VisionSubsystem() {
        limelight = new LimelightDevice("");
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) instance = new VisionSubsystem();
        return instance;
    }
}
