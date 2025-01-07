package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private static VisionSubsystem instance;
    private final ArrayList<LimelightDevice> limelights;

    private VisionSubsystem() {
        limelights = new ArrayList<>();
        for (LimelightConstants limelight : VisionConstants.kLimelights) {
            limelights.add(new LimelightDevice(limelight));
        }
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) instance = new VisionSubsystem();
        return instance;
    }


}
