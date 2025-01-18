package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.spartronics4915.frc2025.Constants.ArmConstants.kArmPIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase { 
    
    private SparkMax mArmMotor;
    private SparkClosedLoopController mArmClosedLoopController;
    private SparkMaxConfig config;

    private TrapezoidProfile mArmProfile;

    private double mCurrentSetPoint = 0.0;
    private State mCurrentState;

    public ArmSubsystem() {
        
        SparkMax mArmMotor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .encoder.positionConversionFactor(ArmConstants.kPositionConversionFactor);
        config
            .encoder.velocityConversionFactor(ArmConstants.kVelocityConversionFactor);
        config
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ArmConstants.kArmPIDConstants.kP, ArmConstants.kArmPIDConstants.kI, ArmConstants.kArmPIDConstants.kD);
    
        mArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private double getPosition() {
        
        double positionFactor = mArmMotor.configAccessor.encoder.getPositionConversionFactor();
        double velocityFactor = mArmMotor.configAccessor.encoder.getVelocityConversionFactor();

        config.signals.primaryEncoderPositionPeriodMs(ArmConstants.kPeriodMs);

        mArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        double position = mArmMotor.getEncoder().getPosition();

        return position;
    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(getPosition(), 0.0);
    }

    @Override
    public void periodic() {
        //need set points as a imput

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State(mCurrentSetPoint, 0.0));

        setVoltage (
            mArmClosedLoopController.setReference(mCurrentSetPoint, ControlType.kCurrent)
        );
    }
    
    private void initClosedLoopController() {
        mArmClosedLoopController = mArmMotor.getClosedLoopController();
    }

    public void moveToIntake() {
    
    }

    public void moveToScore() {

    }

    public void moveToStow() {

    }
}
