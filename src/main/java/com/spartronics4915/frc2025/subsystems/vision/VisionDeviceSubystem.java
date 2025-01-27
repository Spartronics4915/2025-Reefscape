package com.spartronics4915.frc2025.subsystems.vision;

import java.util.Optional;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionDeviceSubystem {
        public ArrayList<Integer> getVisibleTagIDs();
        public Optional<Pose2d> getBotPose2dFromReefCamera();
}       
