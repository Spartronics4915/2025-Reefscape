package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightModel;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightRole;
import com.spartronics4915.frc2025.Constants.VisionConstants.PoseEstimationMethod;
import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.LimelightHelpers.RawFiducial;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    /**
     * Use MegaTag1 if it's within the first three seconds of auto, otherwise use MegaTag2
     */
    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve) {
        if (DriverStation.isAutonomous() && !Robot.AUTO_TIMER.hasElapsed(3)) {
            return getVisionMeasurement(swerve, PoseEstimationMethod.MEGATAG_1);
        }
        return getVisionMeasurement(swerve, PoseEstimationMethod.MEGATAG_2);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve, PoseEstimationMethod method) {
        if (role == LimelightRole.NOTHING || role == LimelightRole.OBSERVER) {
            return Optional.empty();
        }
        LimelightHelpers.PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;
        switch (method) {
            case MEGATAG_1:
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
                break;
            case MEGATAG_2:
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
                stdDevs.get()
                ));
        }
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) return Optional.empty();
        double transStdDev = 0.6; //not very trustworthy to start
        
        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) { //single tag
            transStdDev += 0.2; //trust less if single tag
            RawFiducial singleTag = poseEstimate.rawFiducials[0];
            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); //don't trust if too ambiguous or too far
            }
            if (singleTag.distToCamera < 3) transStdDev -= 0.2; //trust more if close
        } else { //multi tag
            double furthestDistance = Arrays.stream(poseEstimate.rawFiducials)
                                            .map(el -> el.distToCamera)
                                            .max(Double::compare)
                                            .get();
            if (poseEstimate.tagCount > 2) transStdDev -= 0.2; //trust more if more than two tags
            if (furthestDistance > 7) transStdDev += 0.4; //trust less if at least one tag is very far
            if (poseEstimate.avgTagDist < 3) transStdDev -= 0.2; //trust more if tags tend to be close
        }

        double rotStdDev = (DriverStation.isAutonomous() && !Robot.AUTO_TIMER.hasElapsed(3))
                           ? 0.3 //trust a lot if start of auto
                           : Double.MAX_VALUE; //otherwise don't trust at all

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) return Optional.empty();
        if (swerve.getAngularVelocity().abs(Units.DegreesPerSecond) > VisionConstants.kMaxAngularSpeed) 
            return Optional.empty(); //don't trust if turning too fast
        
        double transStdDev = 0.4; //trustworthy to start

        if (poseEstimate.avgTagDist > 8) return Optional.empty(); //don't trust if too far
        transStdDev += poseEstimate.avgTagDist * 0.1; //trust less and less the further away the tags are
        if (poseEstimate.tagCount > 1) transStdDev -= 0.1; //trust slightly more if multiple tags seen

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
