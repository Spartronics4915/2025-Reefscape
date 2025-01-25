package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ElevatorConstants;
import com.spartronics4915.frc2025.Constants.ElevatorConstants.ElevatorSubsystemState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    
    public ElevatorSubsystem() {
        // Main elevator motor init
        SparkMax motor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .inverted(ElevatorConstants.motorInverted)
            .idleMode(IdleMode.kBrake);
        motorConfig.encoder
            .positionConversionFactor(ElevatorConstants.motorPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.motorVelocityConversionFactor);
        motorConfig
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.motorPIDConstants.kP, ElevatorConstants.motorPIDConstants.kI, ElevatorConstants.motorPIDConstants.kD);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower motor init
        SparkMax follower = new SparkMax(ElevatorConstants.elevatorFollowerID, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig
            .inverted(ElevatorConstants.followerInverted)
            .idleMode(IdleMode.kBrake);
        followerConfig.encoder
            .positionConversionFactor(ElevatorConstants.followerPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.followerVelocityConversionFactor);
        followerConfig
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.followerPIDConstants.kP, ElevatorConstants.followerPIDConstants.kI, ElevatorConstants.followerPIDConstants.kD);

        followerConfig.follow(motor);
        motor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void moveToPosition(ElevatorSubsystemState value) {
        
    }
}
