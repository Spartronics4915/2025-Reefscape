package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightModel;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightRole;
import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.LimelightHelpers.RawFiducial;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {

    private final String name;
    private final LimelightModel model;
    private final int id;
    private final LimelightRole role;

    public LimelightDevice(LimelightConstants constants) {
        this.name = "limelight-" + constants.name();
        this.model = constants.model();
        this.id = constants.id();
        this.role = constants.role();

        int portOffset = (id - 11) * 10; // limelight 11 will have 5800-5809, limelight 12 will have 5810-5819, etc.

        // ports are forwarded to http://roborio-4915-FRC.local and are accessed while tethered to the roborio over USB
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port + portOffset, name + ".local", port); 
        }
    }

    public LimelightRole getRole() {
        return role;
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

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve) {
        return getVisionMeasurement(swerve, false);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve, boolean useMegaTag1) {
        if (role == LimelightRole.NOTHING || role == LimelightRole.OBSERVER) {
            return Optional.empty();
        }
        boolean rejectUpdate = false;
        LimelightHelpers.PoseEstimate poseEstimate;
        if (useMegaTag1) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            if (poseEstimate == null) return Optional.empty();
            if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
                RawFiducial singleTag = poseEstimate.rawFiducials[0];
                if (singleTag.ambiguity > 0.7) {
                    rejectUpdate = true;
                }
                if (singleTag.distToCamera > 3) {
                    rejectUpdate = true;
                }
            }
            if (poseEstimate.tagCount == 0) {
                rejectUpdate = true;
            }
        } else {
            LimelightHelpers.SetRobotOrientation(name, swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0);
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            if (poseEstimate == null) return Optional.empty();
            if (swerve.getAngularVelocity().abs(Units.DegreesPerSecond) > VisionConstants.kMaxAngularSpeed) {
                rejectUpdate = true;
            }
            if (poseEstimate.tagCount == 0) {
                rejectUpdate = true;
            }
        }
        if (rejectUpdate) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist,
                useMegaTag1,
                role
                ));
        }
    }

    public Optional<Pose2d> getPose2d() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (pose.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(pose.pose);
    }

    public ArrayList<Integer> getVisibleTags() {
        if (role == LimelightRole.NOTHING || role == LimelightRole.OBSERVER) {
            return new ArrayList<Integer>();
        }
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        ArrayList<Integer> visibleTags = new ArrayList<>();
        for (RawFiducial raw : fiducials) {
            visibleTags.add(raw.id);
        }
        return visibleTags;
    }
}
