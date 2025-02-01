package com.spartronics4915.frc2025.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.Constants;
import com.spartronics4915.frc2025.Constants.ClimberConstants;
import com.spartronics4915.frc2025.Constants.ClimberConstants.ClimberState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {


    private SparkMax mClimberMotor;
    private SparkMaxConfig config;
    private RelativeEncoder mClimberEncoder;
    private TrapezoidProfile mClimberProfile;

    private Rotation2d mCurrentSetPoint = Rotation2d.fromRotations(0);
    private State mCurrentState;
    
    private SparkClosedLoopController mClosedLoopController;
    
    public ClimberSubsystem () {                 //-need fill in motor Ids
        
        mClimberMotor = new SparkMax(1, MotorType.kBrushless);
            config = new SparkMaxConfig();

                config
                    .inverted(true)
                    .idleMode(IdleMode.kBrake);
                config.encoder
                    .positionConversionFactor(1000)
                    .velocityConversionFactor(1000);
                config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(Constants.ClimberConstants.kP, Constants.ClimberConstants.kI, Constants.ClimberConstants.kD);
    
                mClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                mClimberEncoder = mClimberMotor.getEncoder();
        
        initClimberProfile();
        
        initClosedLoopController();

        resetMechanism();
    }

    private Rotation2d convertRaw(double rotation) {
        Rotation2d angle = Rotation2d.fromDegrees(rotation);
        return angle;
    }

    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    public void resetMechanism(){
        var position = getPosition();
        mCurrentSetPoint = (position);
        mCurrentState = new State(angleToRaw(position), 0.0);
    }

    private void setMechanismAngle(Rotation2d angle){
        mClimberEncoder.setPosition(angleToRaw(angle));
        resetMechanism();
    }

    public void setSetpoint(Rotation2d newSetpoint){
        mCurrentSetPoint = newSetpoint;
    }

    public void setSetpoint(ClimberState newState) {
        setSetpoint(newState.angle);
    }

    public void incrementAngle(Rotation2d delta){
        mCurrentSetPoint = mCurrentSetPoint.plus(delta);

    }

    private Rotation2d getPosition() {
        double position = mClimberMotor.getEncoder().getPosition();

        return convertRaw(position);
    }

    private double mCurrentSetpoint = 0.0;

    private void initClosedLoopController() {
        mClosedLoopController = mClimberMotor.getClosedLoopController();
    }

     private void initClimberProfile() {
        mClimberProfile = new TrapezoidProfile(ClimberConstants.kConstraints);
        mCurrentState = new State(angleToRaw(getPosition()), 0.0);
    }

    private double getVelocity() {
        double velocity = mClimberMotor.getEncoder().getVelocity();
        return velocity;
    }

    @Override
    public void periodic() {
        
    }

    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }


}