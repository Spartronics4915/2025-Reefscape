package com.spartronics4915.frc2025.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightModel;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightRole;
import com.spartronics4915.frc2025.Constants.VisionConstants.PoseEstimationMethod;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * this stores all of the records used in the Constants file
 */
public final class Structures {

    public record PIDFConstants(
        double kP, 
        double kI, 
        double kD, 
        double kF
    ) {}

    public record FeedForwardConstants(
        double kS,
        double kG,
        double kV,
        double kA
    ) {}

    public final record MotorConstants(
        int motorID,
        MotorType motorType,
        boolean motorIsInverted,
        IdleMode idleMode,
        int currentLimit
    ) {}

    public final record LimelightConstants(
        String name,
        LimelightModel model,
        int id,
        LimelightRole role
    ) {}

    public static record VisionMeasurement(
        Pose2d pose,
        double timestamp,
        Matrix<N3, N1> stdDevs,
        String diagName,
        int diagTagCount,
        double diagTagDistance,
        double diagRobotSpeed,
        PoseEstimationMethod diagMethod
    ) {}


}
