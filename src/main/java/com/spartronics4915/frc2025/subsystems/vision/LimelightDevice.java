package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.StdDevConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightModel;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightRole;
import com.spartronics4915.frc2025.Constants.VisionConstants.PoseEstimationMethod;
import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.LimelightHelpers.PoseEstimate;
import com.spartronics4915.frc2025.LimelightHelpers.RawFiducial;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.AprilTagRegion;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase {

    private final String name;
    private final LimelightModel model;
    private final int id;
    private final LimelightRole role;
    private int[] tagFilter = new int[]{};

    public LimelightDevice(LimelightConstants constants) {
        this.name = "limelight-" + constants.name();
        this.model = constants.model();
        this.id = constants.id();
        this.role = constants.role();

        // // ports are forwarded to http://roborio-4915-FRC.local and are accessed while tethered to the roborio over USB
        // int portOffset = (id - 11) * 10; // limelight 11 will have 5800-5809, limelight 12 will have 5810-5819, etc.
        // for (int port = 5800; port <= 5809; port++) {
        //     PortForwarder.add(port + portOffset, name + ".local", port); 
        // }
    }

    public void setTagFilter(Optional<DriverStation.Alliance> alliance) {
        AprilTagRegion region;
        switch (role) {
            case REEF:
            region = AprilTagRegion.kReef.and(AprilTagRegion.kProcessor);
            break;
            case ALIGN:
            region = AprilTagRegion.kReef;
            break;
            case STATION:
            region = AprilTagRegion.kStation.and(AprilTagRegion.kBarge);
            break;
            default:
            region = AprilTagRegion.kEmpty;
        }

        if (alliance.isEmpty()) tagFilter = region.both();
        else {
            switch (alliance.get()) {
                case Red:
                    tagFilter = region.red();
                    break;
                case Blue:
                    tagFilter = region.blue();
                    break;
            }
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
        if (role == LimelightRole.NOTHING) return Optional.empty();
        PoseEstimationMethod method = PoseEstimationMethod.MEGATAG_2;
        final boolean BEFORE_MATCH = !Robot.AUTO_TIMER.hasElapsed(0.01) && !Robot.TELEOP_TIMER.hasElapsed(0.01);
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        final boolean twoOrMoreTags = poseEstimate.tagCount >= 2;
        final boolean closeEnough = poseEstimate.avgTagDist < VisionConstants.kMaxDistanceForMegaTag1;
        double robotSpeed = swerve.getSpeed();
        final boolean movingSlowEnough = robotSpeed < VisionConstants.kMaxSpeedForMegaTag1;
        final boolean CAN_GET_GOOD_HEADING = twoOrMoreTags && movingSlowEnough && closeEnough;
        if (BEFORE_MATCH && !CAN_GET_GOOD_HEADING) return Optional.empty(); //we only want the best for our inital pose
        if (CAN_GET_GOOD_HEADING || LimelightVisionSubsystem.getMegaTag1Override()) method = PoseEstimationMethod.MEGATAG_1;
        return getVisionMeasurement(swerve, method);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve, PoseEstimationMethod method) {
        if (role == LimelightRole.NOTHING) return Optional.empty();
        PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;
        switch (method) {
            case MEGATAG_1:
                LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{});
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
                break;
            case MEGATAG_2:
                LimelightHelpers.SetFiducialIDFiltersOverride(name, tagFilter);
                LimelightHelpers.SetRobotOrientation(name, swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0);
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
                stdDevs = calculateStdDevsMegaTag2(poseEstimate, swerve);
                break;
            default:
                System.out.println("Unknown pose estimation method provided: " + method);
                return Optional.empty();
        }
        if (stdDevs.isEmpty()) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                stdDevs.get(),
                name,
                poseEstimate.tagCount,
                poseEstimate.avgTagDist,
                swerve.getSpeed(),
                method
                ));
        }
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        double transStdDev = StdDevConstants.MegaTag1.kInitialValue;
        
        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) { //single tag TODO: why are two checks needed?
            RawFiducial singleTag = poseEstimate.rawFiducials[0];
            if (VisionConstants.kVisionDiagnostics) SmartDashboard.putNumber("VisionDiagnostics/" + name + "/single tag pose ambiguity", singleTag.ambiguity);
            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); //don't trust if too ambiguous or too far
            }
            transStdDev += StdDevConstants.MegaTag1.kSingleTagPunishment; //megatag1 performs much worse with only one tag
        }
        transStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        transStdDev += swerve.getSpeed() * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); //make sure we aren't putting all our trust in vision

        double rotStdDev = 0.3; //we want to get the rotation from megatag1

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        if (swerve.getAngularVelocity().abs(Units.DegreesPerSecond) > VisionConstants.kMaxAngularSpeed) 
            return Optional.empty(); //don't trust if turning too fast
        if (poseEstimate.avgTagDist > 8) return Optional.empty();
        
        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        if (poseEstimate.tagCount > 1) transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus; //TODO: is this even needed?
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += swerve.getSpeed() * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); //make sure we aren't putting all our trust in vision

        double rotStdDev = Double.MAX_VALUE; //never trust rotation under any circumstances

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    public Optional<Pose2d> getPose2d() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (pose.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(pose.pose);
    }

    public ArrayList<Integer> getVisibleTags() {
        if (role == LimelightRole.NOTHING) return new ArrayList<Integer>();
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        ArrayList<Integer> visibleTags = new ArrayList<>();
        for (RawFiducial raw : fiducials) {
            visibleTags.add(raw.id);
        }
        return visibleTags;
    }
}
