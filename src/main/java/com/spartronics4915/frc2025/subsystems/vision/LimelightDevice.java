package com.spartronics4915.frc2025.subsystems.vision;

import com.spartronics4915.frc2025.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {
    private final String name;

    public LimelightDevice(String name) {
        this.name = name;
    }

    public double getTx() {
        return LimelightHelpers.getTX(name);
    }

    public double getTy() {
        return LimelightHelpers.getTY(name);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }
}
