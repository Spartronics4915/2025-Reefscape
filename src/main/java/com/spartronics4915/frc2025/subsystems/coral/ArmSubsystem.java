package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ArmConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase { 

    private TrapezoidProfile mArmProfile;

    private double mCurrentSetPoint = 0.0;
    private State mCurrentState;

    public ArmSubsystem() {

    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(getArmPosition(), 0.0);
    }

    @Override
    public void periodic() {
        //need set points as a imput

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State(mCurrentSetPoint, 0.0));

        
    }
    
    public void moveToIntake() {
    
    }

    public void moveToScore() {

    }

    public void moveToStow() {

    }
}
