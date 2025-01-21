package com.spartronics4915.frc2025.subsystems;

import java.util.Optional;

import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;

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
        
    }

    private Optional<Pose2d> getVisionPose() {
        
    }

    /**
     * 
     * @return Robot pose from the swerve drive, or the robot pose from vision if the distance between them is too great
     */
    public Pose2d getPose() {
        
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
