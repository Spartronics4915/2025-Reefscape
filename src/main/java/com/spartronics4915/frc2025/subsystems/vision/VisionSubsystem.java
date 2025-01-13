package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.VisionMeasurement;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

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
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    public ArrayList<VisionMeasurement> getVisionMeasurements(SwerveDrive swerve) {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        limelights.forEach((limelight) -> {
            Optional<VisionMeasurement> measurement = limelight.getVisionMeasurement(swerve);
            if (measurement.isPresent()) {
                measurements.add(measurement.get());
            }
        });
        return measurements;
    }

    public ArrayList<Integer> getVisibleTagIDs() {
        Set<Integer> set = new HashSet<>();
        limelights.forEach((limelight) -> {
            ArrayList<Integer> results = limelight.getVisibleTags();
            set.addAll(results);
        });
        return new ArrayList<Integer>(set);
    }

    public ArrayList<Pose3d> getVisibleTagPoses() {
        ArrayList<Integer> visibleTagIDs = getVisibleTagIDs();
        ArrayList<Pose3d> visibleTagPoses = new ArrayList<>();
        visibleTagIDs.forEach((Integer tagID) -> {
            Optional<Pose3d> tagPose = VisionConstants.kFieldLayout.getTagPose(tagID);
            if (tagPose.isEmpty()) {
                Logger.getLogger(this.getClass().getName()).warning("Tag ID " + tagID + " not found in field layout " + VisionConstants.kFieldLayout);
            } else {
                visibleTagPoses.add(tagPose.get());
            }
        });
        return visibleTagPoses;
    }
}
