package com.spartronics4915.frc2025.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.LimelightModel;

/**
 * this stores all of the records used in the Constants file
 */
public final class Structures {
    public record PIDConstants(
        double kP, 
        double kI, 
        double kD
    ) {}

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
        int id
    ) {}


}
