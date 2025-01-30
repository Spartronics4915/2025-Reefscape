package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ElevatorConstants;
import com.spartronics4915.frc2025.Constants.ElevatorConstants.ElevatorSubsystemState;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorSubsystem extends SubsystemBase implements ModeSwitchInterface{

    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkMax follower;
    private RelativeEncoder motorEncoder;
    private RelativeEncoder followerEncoder;

    private double currentSetPoint;
    
    public ElevatorSubsystem() {
        // Main elevator motor init
        motor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

        RelativeEncoder motorEncoder = motor.getEncoder();

        motorConfig = new SparkMaxConfig();

        motorConfig
            .inverted(ElevatorConstants.motorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.motorSmartCurrentLimit)
            .secondaryCurrentLimit(ElevatorConstants.motorSecondaryCurrentLimit);
        motorConfig.encoder
            .positionConversionFactor(ElevatorConstants.motorPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.motorVelocityConversionFactor);
        motorConfig
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.motorPIDConstants.kP, ElevatorConstants.motorPIDConstants.kI, ElevatorConstants.motorPIDConstants.kD);
            
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower motor init
        follower = new SparkMax(ElevatorConstants.elevatorFollowerID, MotorType.kBrushless);

        RelativeEncoder followerEncoder = follower.getEncoder();

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        followerConfig
            .inverted(ElevatorConstants.followerInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.followerSmartCurrentLimit)
            .secondaryCurrentLimit(ElevatorConstants.followerSecondaryCurrentLimit);
        followerConfig.encoder
            .positionConversionFactor(ElevatorConstants.followerPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.followerVelocityConversionFactor);
        followerConfig
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.followerPIDConstants.kP, ElevatorConstants.followerPIDConstants.kI, ElevatorConstants.followerPIDConstants.kD);

        followerConfig.follow(motor);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetMechanism();
    }

    public void resetMechanism() {
        double position = getPosition();
        currentSetPoint = position;
    }

    private double getPosition() {
        return motor.getEncoder().getPosition();
    }

    private double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        currentSetPoint = MathUtil.clamp(currentSetPoint, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    }

    public void moveToPosition(ElevatorSubsystemState value) {
        currentSetPoint = value.meter;
    }

    public void setSetPoint(double setPoint) {
        currentSetPoint = setPoint;
    }

    private double angleToRaw(Rotation2d angle) {
        return angle.getRotations();
    }

    private void setMechanismAngle(Rotation2d angle){
        motorEncoder.setPosition(angleToRaw(angle));
        followerEncoder.setPosition(angleToRaw(angle));
        resetMechanism();
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }
}
