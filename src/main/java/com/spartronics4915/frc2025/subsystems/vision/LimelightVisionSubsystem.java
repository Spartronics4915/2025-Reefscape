package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.LimelightRole;
import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.VisionMeasurement;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionDeviceSubystem {
    private final ArrayList<LimelightDevice> limelights;
    private final SwerveSubsystem swerveSubsystem;
    private final StructArrayPublisher<Pose3d> visionTargetPublisher;
    private final AprilTagFieldLayout fieldLayout;

    public LimelightVisionSubsystem(SwerveSubsystem swerveSubsystem, AprilTagFieldLayout fieldLayout) {
        limelights = new ArrayList<>();
        for (LimelightConstants limelight : VisionConstants.kLimelights) {
            limelights.add(new LimelightDevice(limelight));
        }

        this.fieldLayout = fieldLayout;
        this.swerveSubsystem = swerveSubsystem;

        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        StructArrayTopic<Pose3d> visionTargetTopic = networkTableInstance.getStructArrayTopic("vision targets",
                Pose3d.struct);
        visionTargetPublisher = visionTargetTopic.publish();
        Shuffleboard.getTab("logging").addString("vision target ids", () -> this.getVisibleTagIDs().toString());
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
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);
            if (tagPose.isEmpty()) {
                Logger.getLogger(this.getClass().getName())
                        .warning("Tag ID " + tagID + " not found in field layout ");
            } else {
                visibleTagPoses.add(tagPose.get());
            }
        });
        return visibleTagPoses;
    }

    private LimelightDevice getLimelightFromRole(LimelightRole role) {
        return limelights.stream()
                .filter(limelight -> limelight.getRole() == role)
                .findAny()
                .orElse(null);
    }

    @Override
    public void periodic() {
        visionTargetPublisher.set(getVisibleTagPoses().toArray(new Pose3d[0]));
    }

    public Optional<Pose2d> getBotPose2dFromReefCamera() {
        LimelightDevice reef = getLimelightFromRole(LimelightRole.REEF);
        if (reef != null) return reef.getPose2d();
        return Optional.empty();
    }
}
