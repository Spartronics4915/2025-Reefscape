package com.spartronics4915.frc2025.subsystems.vision;

import java.util.Optional;

import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class LimelightDevice extends SubsystemBase {

    public static record VisionMeasurement(Pose2d pose, double timestamp) {}

    private final String name;

    public LimelightDevice(String name) {
        this.name = "limelight-" + name;
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
}
