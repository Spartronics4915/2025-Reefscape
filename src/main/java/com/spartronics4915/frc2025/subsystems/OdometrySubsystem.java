package com.spartronics4915.frc2025.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.Constants.OdometryConstants;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
    private VisionDeviceSubystem visionSubsystem;
    private SwerveSubsystem swerveSubsystem;

    public OdometrySubsystem(VisionDeviceSubystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    public Optional<Double> getVisionStdDevs() {
        return null; //placeholder so code can compile
    }

    private Optional<Pose2d> getVisionPose() {
        if (Robot.isSimulation()) {
            return visionSubsystem.getBotPose2dFromReefCamera();
        }
        ArrayList<VisionMeasurement> visionMeasurements = ((LimelightVisionSubsystem) visionSubsystem).getVisionMeasurements();
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


    private void updateSwervePoseEstimator(Pose2d pose, double stdDev) {
        
    }

    @Override
    public void periodic() {
        //query vision
        //...
        Optional<Double> calculatedStdDevs = getVisionStdDevs();
        if (calculatedStdDevs.isEmpty()) return; //Optional.empty() = "don't bother"
        double stdDevs = calculatedStdDevs.get();
        //...
        //update pose estimator
    }
}
