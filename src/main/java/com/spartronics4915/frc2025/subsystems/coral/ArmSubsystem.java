package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.spartronics4915.frc2025.Constants.ArmConstants.kArmPIDConstants;
import com.spartronics4915.frc2025.util.Structures.FeedForwardConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase { 
    
    private SparkMax mArmMotor;
    private SparkClosedLoopController mArmClosedLoopController;
    private SparkMaxConfig config;
    private ArmFeedforward mFFCalculator;

    private TrapezoidProfile mArmProfile;

    private double mCurrentSetPoint = 0.0;
    private State mCurrentState;

    private double velocity;

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

        mFFCalculator = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
        
    }

    private Rotation2d mConvertRaw(double rotation) {
        Rotation2d angle = Rotation2d.fromDegrees(rotation);
        return angle;
    }

    private double mAngleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    private double getPosition() {
        double position = mArmMotor.getEncoder().getPosition();

        return position;
    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(getPosition(), velocity);
    }

    private double getVelocity() {
        double velocity = mArmMotor.getEncoder().getVelocity();

        return velocity;
    }

    @Override
    public void periodic() {
        //need set points as a imput
        mCurrentSetPoint = Rotation2d.fromRotations(
            MathUtil.clamp(mCurrentSetPoint.getRotations(), ArmConstants.kMinAngle.getRotations(), ArmConstants.kMaxAngle.getRotations()));

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State((mCurrentSetPoint), velocity));

        mArmClosedLoopController.setReference(mCurrentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, mFFCalculator.calculate(getPosition(), getVelocity()));

    }
    
    private void initClosedLoopController() {
        mArmClosedLoopController = mArmMotor.getClosedLoopController();
    }

    public void moveToIntake() {
    
    }

    public void moveToLowScore() {

    }

    public void moveToHighScore() {

    }

    public void moveToStow() {

    }
}
