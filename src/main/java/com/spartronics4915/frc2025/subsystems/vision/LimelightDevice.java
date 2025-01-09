package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.LimelightHelpers.RawFiducial;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class LimelightDevice extends SubsystemBase {

    public static record VisionMeasurement(Pose2d pose, double timestamp) {}

    public enum LimelightModel {
        LIMELIGHT_3, LIMELIGHT_3G
    }

    public enum LimelightRole {
        NOTHING, REEF, STATION, OBSERVER
    }

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

    public double getTx() {
        return LimelightHelpers.getTX(name);
    }

    public double getTy() {
        return LimelightHelpers.getTY(name);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveDrive swerve) {
        if (role == LimelightRole.NOTHING || role == LimelightRole.OBSERVER) {
            return Optional.empty();
        }
        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(name, swerve.getOdometryHeading().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (swerve.getGyro().getYawAngularVelocity().abs(Units.DegreesPerSecond) > VisionConstants.kMaxAngularSpeed) {
            rejectUpdate = true;
        }
        if (megaTag2.tagCount == 0) {
            rejectUpdate = true;
        }
        if (rejectUpdate) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds));
        }
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
