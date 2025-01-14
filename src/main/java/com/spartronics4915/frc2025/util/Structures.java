package com.spartronics4915.frc2025.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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


}
