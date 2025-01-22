package com.spartronics4915.frc2025.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.Constants.OdometryConstants;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
    private final VisionDeviceSubystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();

    public OdometrySubsystem(VisionDeviceSubystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    private void updateVisionMeasurements() {
        if (Robot.isReal()) {
            visionMeasurements = ((LimelightVisionSubsystem) visionSubsystem).getVisionMeasurements();
        }
    }

    public Optional<Matrix<N3, N1>> getVisionStdDevs(VisionMeasurement measurement) {
        double sigmaX = 0.0;
        double sigmaY = 0.0;
        double sigmaTheta = 0.0;
        return Optional.of(MatBuilder.fill(Nat.N3(), Nat.N1(), sigmaX, sigmaY, sigmaTheta));
    }

    private Optional<Pose2d> getVisionPose() {
        if (Robot.isSimulation()) {
            return visionSubsystem.getBotPose2dFromReefCamera();
        }
        if (visionMeasurements.size() == 0) return Optional.empty();
        if (visionMeasurements.size() == 1) return Optional.of(visionMeasurements.get(0).pose());
        return Optional.of(
            visionMeasurements.get(0).pose().interpolate(
                visionMeasurements.get(1).pose(),
                0.5)
        );
    }

    /**
     * 
     * @return Robot pose from the swerve drive, or the robot pose from vision if the distance between them is too great
     */
    public Pose2d getPose() {
        Pose2d swervePose = swerveSubsystem.getPose();
        Optional<Pose2d> potentialVisionPose = getVisionPose();
        if (potentialVisionPose.isEmpty()) return swervePose;
        Pose2d visionPose = potentialVisionPose.get();
        double distance = swervePose.getTranslation().getDistance(visionPose.getTranslation());
        if (distance > OdometryConstants.kMaxSwerveVisionPoseDifference) return visionPose;
        return swervePose;
    }

    @Override
    public void periodic() {
        updateVisionMeasurements();
        visionMeasurements.forEach((measurement) -> {
            Optional<Matrix<N3, N1>> potentialStdDevs = getVisionStdDevs(measurement);
            if (potentialStdDevs.isPresent()) {
                Matrix<N3, N1> stdDevs = potentialStdDevs.get();
                swerveSubsystem.addVisionMeasurement(measurement.pose(), measurement.timestamp(), stdDevs);
            }
        });
    }
}
