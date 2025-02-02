package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.spartronics4915.frc2025.Constants.ArmConstants.ArmSubsystemState;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class ArmSubsystem extends SubsystemBase implements ModeSwitchInterface{ 
    
    private SparkMax mArmMotor;
    private SparkClosedLoopController mArmClosedLoopController;
    private SparkMaxConfig config;
    private ArmFeedforward mFFCalculator;
    private RelativeEncoder mArmEncoder;

    private TrapezoidProfile mArmProfile;

    private Rotation2d mCurrentSetPoint = Rotation2d.fromRotations(0);;
    private State mCurrentState;

    public ArmSubsystem() {
        
        mArmMotor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);

        mArmEncoder = mArmMotor.getEncoder();
        
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
        
        initArmProfile();

        initClosedLoopController();

        resetMechanism();
    }

    public void resetMechanism(){
        var position = getPosition();
        mCurrentSetPoint = (position);
        mCurrentState = new State(angleToRaw(position), 0.0);
    }

    private Rotation2d convertRaw(double rotation) {
        Rotation2d angle = Rotation2d.fromDegrees(rotation);
        return angle;
    }

    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    private Rotation2d getPosition() {
        double position = mArmMotor.getEncoder().getPosition();

        return convertRaw(position);
    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(angleToRaw(getPosition()), 0.0);
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

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State((angleToRaw(mCurrentSetPoint)), 0.0));

        mArmClosedLoopController.setReference(mCurrentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, mFFCalculator.calculate(mCurrentState.position, mCurrentState.velocity));

    }
    
    private void setMechanismAngle(Rotation2d angle){
        mArmEncoder.setPosition(angleToRaw(angle));
        resetMechanism();
    }

    private void initClosedLoopController() {
        mArmClosedLoopController = mArmMotor.getClosedLoopController();
    }

    public void setSetpoint(Rotation2d newSetpoint){
        mCurrentSetPoint = newSetpoint;
    }

    public void setSetpoint(ArmSubsystemState newState) {
        setSetpoint(newState.angle);
    }

    public void incrementAngle(Rotation2d delta){
        mCurrentSetPoint = mCurrentSetPoint.plus(delta);
    }


    //Commands 

    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(ArmSubsystemState preset){
        return setSetpointCommand(preset.angle);
    }

    public Command setMechanismAngleCommand(Rotation2d newAngle){
        return this.runOnce(() -> setMechanismAngle(newAngle));
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }
}