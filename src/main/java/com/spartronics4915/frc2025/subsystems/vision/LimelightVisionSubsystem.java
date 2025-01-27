package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionDeviceSubystem, ModeSwitchInterface {
    private final ArrayList<LimelightDevice> limelights;

    private LimelightDevice reefLL;
    private LimelightDevice stationLL;
    private LimelightDevice observerLL;

    private final SwerveSubsystem swerveSubsystem;
    private final StructArrayPublisher<Pose3d> visionTargetPublisher;
    private final AprilTagFieldLayout fieldLayout;

    public LimelightVisionSubsystem(SwerveSubsystem swerveSubsystem, AprilTagFieldLayout fieldLayout) {
        limelights = new ArrayList<>();
        for (LimelightConstants config : VisionConstants.kLimelights) {
            LimelightDevice limelight = new LimelightDevice(config);
            limelights.add(limelight);
            boolean diagnosticsNeeded = VisionConstants.kVisionMeasurementDiagnostics;
            switch (config.role()) {
                case REEF:
                    reefLL = limelight;
                    System.out.println("Setting reef limelight to " + config.id());
                    break;
                case STATION:
                    stationLL = limelight;
                    System.out.println("Setting station limelight to " + config.id());
                    break;
                case OBSERVER:
                    observerLL = limelight;
                    diagnosticsNeeded = false;
                    System.out.println("Setting observer limelight to " + config.id());
                    break;
                default:
                    diagnosticsNeeded = false;
                    System.out.println("Not setting " + config.id() + " to anything");
            }
            if (diagnosticsNeeded) {
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + config.name() + "/stddev", -1);
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + config.name() + "/count", -1);
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + config.name() + "/distance", -1);
            }
        }

        updateTagFilters();

        this.fieldLayout = fieldLayout;
        this.swerveSubsystem = swerveSubsystem;

        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        StructArrayTopic<Pose3d> visionTargetTopic = networkTableInstance.getStructArrayTopic("vision targets",
                Pose3d.struct);
        visionTargetPublisher = visionTargetTopic.publish();
        Shuffleboard.getTab("logging").addString("vision target ids", () -> this.getVisibleTagIDs().toString());
    }

    private void updateTagFilters() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        limelights.forEach(limelight -> {
            limelight.setTagFilter(alliance);
        });
    }

    public ArrayList<VisionMeasurement> getVisionMeasurements() {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        limelights.forEach((limelight) -> {
            Optional<VisionMeasurement> measurement = limelight.getVisionMeasurement(swerveSubsystem);
            if (measurement.isPresent()) {
                measurements.add(measurement.get());
            }
        });
        return measurements;
    }

    public boolean canSeeTags() {
        return reefLL.getTv() || stationLL.getTv();
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

    @Override
    public void periodic() {
        getVisionMeasurements().forEach((measurement) -> {
            if (VisionConstants.kVisionMeasurementDiagnostics) {
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + measurement.diagName() + "/stddev", measurement.stdDevs().get(0, 0));
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + measurement.diagName() + "/count", measurement.diagTagCount());
                SmartDashboard.putNumber("VisionMeasurementDiagnostics/" + measurement.diagName() + "/distance", measurement.diagTagDistance());
            }
            swerveSubsystem.addVisionMeasurement(measurement.pose(), measurement.timestamp(), measurement.stdDevs());
        });

        visionTargetPublisher.set(getVisibleTagPoses().toArray(new Pose3d[0]));
    }

    @Override
    public void onModeSwitch() {
        updateTagFilters();
    }

    public Optional<Pose2d> getBotPose2dFromReefCamera() {
        return reefLL.getPose2d();
    }

    public LimelightDevice getReefLimelight() {
        return reefLL;
    }

    public LimelightDevice getStationLimelight() {
        return stationLL;
    }

    public LimelightDevice getObserverLimelight() {
        return observerLL;
    }
}
